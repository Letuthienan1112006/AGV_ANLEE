"""Runs the built TensorRT engine on the Jetson itself.

Separate from yolo26_unified.py (laptop/training side, needs ultralytics,
needs Python 3.8+) and yolo26_raw_decode.py (the postprocessing math,
pure numpy, shared by both sides). This file is the third piece: getting
bytes onto and off the GPU on THIS Jetson's JetPack 4.6.3/Python 3.6.9,
where neither ultralytics nor pycuda are installable (pycuda needs a
source compile against libboost-python, which isn't installed and wasn't
worth the disk/time on a device already at 83% root-disk use - this uses
ctypes against libcudart directly instead, which needs nothing extra).

Status 2026-09-16: engine built and benchmarked via trtexec (FP32:
12.1 qps measured on this Jetson), raw output matched against ONNX, and
this module has run end-to-end through patrol_robot.py on the vehicle.
"""
import ctypes
import glob
import os
import time

import numpy as np
import tensorrt as trt

from yolo26_raw_decode import infer_from_raw_outputs, letterbox_image

CUDA_MEMCPY_HOST_TO_DEVICE = 1
CUDA_MEMCPY_DEVICE_TO_HOST = 2

_TRT_TO_NP = {
    trt.DataType.FLOAT: np.float32,
    trt.DataType.HALF:  np.float16,
    trt.DataType.INT32: np.int32,
    trt.DataType.INT8:  np.int8,
    trt.DataType.BOOL:  np.bool_,
}


def _load_libcudart():
    # Path is JetPack-version-specific (this Jetson: CUDA 10.2); glob so
    # the module doesn't hardcode a version that a JetPack upgrade breaks.
    candidates = (glob.glob("/usr/local/cuda*/targets/*/lib/libcudart.so*")
                 or glob.glob("/usr/lib/*/libcudart.so*"))
    if not candidates:
        raise RuntimeError("libcudart.so not found - is CUDA installed?")
    return ctypes.CDLL(sorted(candidates)[0])


class CudaBuffer:
    """One device allocation plus its paired host numpy array."""

    def __init__(self, cudart, shape, dtype):
        self.cudart = cudart
        self.host = np.empty(shape, dtype=dtype)
        self.device = ctypes.c_void_p()
        ret = cudart.cudaMalloc(ctypes.byref(self.device),
                                ctypes.c_size_t(self.host.nbytes))
        if ret != 0:
            raise RuntimeError("cudaMalloc failed, code {}".format(ret))

    def to_device(self):
        ret = self.cudart.cudaMemcpy(
            self.device, self.host.ctypes.data_as(ctypes.c_void_p),
            ctypes.c_size_t(self.host.nbytes),
            ctypes.c_int(CUDA_MEMCPY_HOST_TO_DEVICE))
        if ret != 0:
            raise RuntimeError("cudaMemcpy H2D failed, code {}".format(ret))

    def to_host(self):
        ret = self.cudart.cudaMemcpy(
            self.host.ctypes.data_as(ctypes.c_void_p), self.device,
            ctypes.c_size_t(self.host.nbytes),
            ctypes.c_int(CUDA_MEMCPY_DEVICE_TO_HOST))
        if ret != 0:
            raise RuntimeError("cudaMemcpy D2H failed, code {}".format(ret))

    def free(self):
        self.cudart.cudaFree(self.device)


class TensorRTUnifiedYOLO26:
    """Same (pred_mask, persons) contract as yolo26_unified.UnifiedYOLO26,
    but running the compiled .engine directly - no ultralytics, no torch."""

    def __init__(self, engine_path, conf_thres=0.4, iou_thres=0.7,
                 min_area=1500, lane_conf_thres=None):
        self.conf_thres = conf_thres
        self.lane_conf_thres = lane_conf_thres
        self.iou_thres = iou_thres
        self.min_area = min_area
        self.cudart = _load_libcudart()

        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError("failed to deserialize engine: " + engine_path)
        self.context = self.engine.create_execution_context()

        self.buffers = {}
        self.input_names = []
        self.output_names = []
        for i in range(self.engine.num_bindings):
            name = self.engine.get_binding_name(i)
            shape = tuple(self.engine.get_binding_shape(i))
            dtype = _TRT_TO_NP[self.engine.get_binding_dtype(i)]
            self.buffers[name] = CudaBuffer(self.cudart, shape, dtype)
            (self.input_names if self.engine.binding_is_input(i)
             else self.output_names).append(name)
        self.input_name = self.input_names[0]
        # The engine's own binding shape is the single source of truth for
        # the input size: re-exporting the model at another resolution then
        # cannot leave a stale constant somewhere disagreeing with the
        # engine actually loaded. (1, 3, H, W)
        _, _, self.model_h, self.model_w = self.buffers[self.input_name].host.shape
        # Log the shape ACTUALLY loaded, not the one a caller thinks it
        # asked for - this is the guard against "dead config family"
        # (KNOWN_ISSUES.md 2026-09-10): AGV_UNIFIED_ENGINE_PATH can point
        # at any .engine file, and only this line proves which one is
        # really running.
        print("[TRT] engine=%s input=(%d, %d) outputs=%s" % (
            os.path.basename(engine_path), self.model_h, self.model_w,
            ", ".join("%s:%s" % (name, self.buffers[name].host.shape)
                      for name in self.output_names)))

    def warmup(self, height=360, width=640):
        dummy = np.zeros((height, width, 3), dtype=np.uint8)
        self.infer(dummy)

    def _execute(self, blob=None):
        started = time.perf_counter()
        if blob is not None:
            self.buffers[self.input_name].host[...] = blob
        for name in self.input_names:
            self.buffers[name].to_device()
        h2d_done = time.perf_counter()
        bindings = [self.buffers[self.engine.get_binding_name(i)].device.value
                   for i in range(self.engine.num_bindings)]
        ok = self.context.execute_v2(bindings)
        if not ok:
            raise RuntimeError("TensorRT execute_v2 failed")
        execute_done = time.perf_counter()
        for name in self.output_names:
            self.buffers[name].to_host()
        d2h_done = time.perf_counter()
        self.last_execute_timing_ms = {
            "h2d": (h2d_done - started) * 1000.0,
            "execute": (execute_done - h2d_done) * 1000.0,
            "d2h": (d2h_done - execute_done) * 1000.0,
        }
        return {name: self.buffers[name].host for name in self.output_names}

    def infer(self, frame_bgr):
        started = time.perf_counter()
        h, w = frame_bgr.shape[:2]
        # Preprocess directly into the reusable TensorRT host buffer. Besides
        # avoiding one full input copy, this keeps allocation pressure out of
        # the control loop on the 4 GB Jetson.
        _, ratio, pad_x, pad_y = letterbox_image(
            frame_bgr,
            new_h=self.model_h, new_w=self.model_w,
            destination=self.buffers[self.input_name].host,
        )
        pre_done = time.perf_counter()
        outputs = self._execute()
        trt_done = time.perf_counter()
        # output0 is (1,41,8400): box+cls+mask coeffs. output1 is the
        # (1,32,Hp,Wp) prototypes - the smaller tensor by element count.
        out0_name = max(self.output_names, key=lambda n: outputs[n].shape[-1])
        out1_name = next(n for n in self.output_names if n != out0_name)
        result = infer_from_raw_outputs(
            outputs[out0_name], outputs[out1_name], h, w, ratio, pad_x, pad_y,
            conf_thres=self.conf_thres, iou_thres=self.iou_thres,
            min_area=self.min_area,
            model_h=self.model_h, model_w=self.model_w,
            lane_conf_thres=self.lane_conf_thres)
        post_done = time.perf_counter()
        self.last_timing_ms = {
            "pre": (pre_done - started) * 1000.0,
            "trt": (trt_done - pre_done) * 1000.0,
            "post": (post_done - trt_done) * 1000.0,
            "total": (post_done - started) * 1000.0,
        }
        self.last_timing_ms.update(self.last_execute_timing_ms)
        return result

    def close(self):
        for buf in self.buffers.values():
            buf.free()
