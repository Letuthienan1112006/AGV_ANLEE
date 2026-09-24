import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models.segmentation.deeplabv3 import DeepLabHead
from torchvision import models
from torchvision.models._utils import IntermediateLayerGetter


class DeepLabV3(nn.Module):
    def __init__(self, num_classes=2, backbone='resnet50', pretrained=True, output_stride=8):
        super(DeepLabV3, self).__init__()

        # --------- chọn backbone ----------
        # Dung API cu (pretrained=bool) cho CA 3 backbone, khong dung
        # weights=... - vi torchvision cu tren Jetson (0.11.1) khong ho tro
        # weights= (loi "unexpected keyword argument 'weights'" trong
        # __init__ cua ResNet/MobileNetV2). API cu van dung tren torchvision
        # moi tren laptop (chi bi deprecated, khong loi) nen dung chung duoc
        # ca 2 may. Dung nham API moi tung lam hong resnet50 tren Jetson
        # 2026-09-04 khi sua rieng nhanh mobilenetv2 - dung lap lai.
        if backbone == 'resnet50':
            base_model = models.resnet50(pretrained=pretrained)
            return_layers = {'layer4': 'out'}
            in_channels = 2048

        elif backbone == 'resnet101':
            base_model = models.resnet101(pretrained=pretrained)
            return_layers = {'layer4': 'out'}
            in_channels = 2048

        elif backbone == 'mobilenetv2':
            base_model = models.mobilenet_v2(pretrained=pretrained).features
            return_layers = {'17': 'out'}  # layer '18' la conv mo rong cuoi (1280 kenh), khong khop in_channels=320
            in_channels = 320

        else:
            raise ValueError(f"Backbone '{backbone}' không được hỗ trợ. Hãy dùng 'resnet50', 'resnet101' hoặc 'mobilenetv2'.")

        # --------- trích xuất đặc trưng ----------
        self.backbone = IntermediateLayerGetter(base_model, return_layers)
        self.classifier = DeepLabHead(in_channels, num_classes)

    def forward(self, x):
        input_shape = x.shape[-2:]
        features = self.backbone(x)
        x = self.classifier(features["out"])
        x = F.interpolate(x, size=input_shape, mode='bilinear', align_corners=False)
        return x


def create_deeplabv3(num_classes=2, backbone='resnet50', pretrained=True, output_stride=8):
    """Tạo mô hình DeepLabV3 với backbone và số lớp tùy chỉnh."""
    model = DeepLabV3(
        num_classes=num_classes,
        backbone=backbone,
        pretrained=pretrained,
        output_stride=output_stride,
    )
    return model


def test():
    """Kiểm thử nhanh mô hình DeepLabV3."""
    x = torch.randn((2, 3, 360, 640))
    model = create_deeplabv3(num_classes=2)
    preds = model(x)
    print(f"Input shape: {x.shape}")
    print(f"Output shape: {preds.shape}")
    assert preds.shape == (2, 2, 360, 640), f"Expected (2, 2, 360, 640), got {preds.shape}"
    print("✅ Test passed! Mô hình hoạt động ổn định.")


if __name__ == "__main__":
    test()
