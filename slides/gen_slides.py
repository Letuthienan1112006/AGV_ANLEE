#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generator cho bo slide bao cao AGV (10 artboard .dc.html)."""
import os

OUT = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- helmet/css
FONTS = '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600;700&family=IBM+Plex+Mono:wght@400;500;600;700&display=swap">'

BASE_CSS = """
:root{
  --bg:oklch(15% 0.025 255);
  --surface:oklch(19.5% 0.03 255);
  --surface-2:oklch(24% 0.03 255);
  --border:oklch(33% 0.03 255);
  --text:oklch(95% 0.008 255);
  --text-dim:oklch(68% 0.02 255);
  --text-faint:oklch(50% 0.02 255);
  --accent:oklch(78% 0.15 75);
  --accent-soft:oklch(78% 0.15 75 / 0.14);
  --accent-line:oklch(78% 0.15 75 / 0.35);
  --good:oklch(74% 0.15 152);
  --good-soft:oklch(74% 0.15 152 / 0.14);
  --bad:oklch(68% 0.17 25);
  --bad-soft:oklch(68% 0.17 25 / 0.14);
}
*{box-sizing:border-box;}
html,body{margin:0;padding:0;}
body{
  width:1280px;height:720px;overflow:hidden;
  background:var(--bg);
  color:var(--text);
  font-family:'IBM Plex Sans',system-ui,sans-serif;
  position:relative;
}
.mono{font-family:'IBM Plex Mono',ui-monospace,monospace;}
.slide{
  width:1280px;height:720px;
  display:flex;flex-direction:column;
  padding:48px 64px 40px 64px;
  position:relative;
}
a{color:var(--accent);}
a:hover{color:oklch(85% 0.15 75);}

/* corner brand mark - required on every slide */
.islab{
  position:absolute; top:32px; right:64px;
  display:flex; align-items:center; gap:8px;
  font-family:'IBM Plex Mono',monospace;
  font-size:13px; font-weight:600; letter-spacing:0.18em;
  color:var(--accent); text-transform:uppercase;
}
.islab-dot{width:7px;height:7px;background:var(--accent);flex:none;}

/* footer chrome */
.foot{
  position:absolute; left:64px; right:64px; bottom:28px;
  display:flex; justify-content:space-between; align-items:flex-end;
  font-family:'IBM Plex Mono',monospace; font-size:11px;
  color:var(--text-faint); letter-spacing:0.04em;
}
.foot-page{color:var(--text-dim);}

.eyebrow{
  font-family:'IBM Plex Mono',monospace; font-size:13px; font-weight:600;
  letter-spacing:0.16em; text-transform:uppercase; color:var(--accent);
  margin:0 0 10px 0;
}
.h1{font-size:40px;font-weight:700;line-height:1.12;margin:0 0 6px 0;letter-spacing:-0.01em;text-wrap:balance;}
.h2{font-size:26px;font-weight:600;line-height:1.2;margin:0 0 4px 0;text-wrap:balance;}
.lede{font-size:16px;line-height:1.55;color:var(--text-dim);margin:0;max-width:920px;}

.rule{height:1px;background:var(--border);border:none;margin:20px 0;}

.card{
  background:var(--surface);
  border:1px solid var(--border);
  border-radius:6px;
  padding:20px 22px;
}
.tile{
  background:var(--surface);
  border:1px solid var(--border);
  border-radius:6px;
  padding:18px 20px;
  display:flex; flex-direction:column; gap:6px;
}
.tile-label{font-size:12px;color:var(--text-dim);letter-spacing:0.03em;}
.tile-num{font-family:'IBM Plex Mono',monospace;font-size:30px;font-weight:600;color:var(--text);}
.tile-sub{font-size:12.5px;color:var(--text-faint);}
.good{color:var(--good);}
.text-dim{color:var(--text-dim);}
.bad{color:var(--bad);}
.accent{color:var(--accent);}

.arrow{color:var(--text-faint);}
.pill{
  display:inline-flex;align-items:center;gap:6px;
  font-family:'IBM Plex Mono',monospace;font-size:11.5px;font-weight:600;
  letter-spacing:0.04em;text-transform:uppercase;
  padding:4px 10px;border-radius:4px;
}
.pill-good{background:var(--good-soft);color:var(--good);}
.pill-bad{background:var(--bad-soft);color:var(--bad);}
.pill-accent{background:var(--accent-soft);color:var(--accent);}

ul.clean{list-style:none;margin:0;padding:0;display:flex;flex-direction:column;gap:10px;}
"""

def svg_icon(kind, size=20, color="var(--accent)"):
    s = size
    common = f'width="{s}" height="{s}" viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"'
    icons = {
        "check": f'<svg {common}><path d="M4 12.5l5 5L20 6.5"/></svg>',
        "cross": f'<svg {common}><path d="M6 6l12 12M18 6L6 18"/></svg>',
        "clock": f'<svg {common}><circle cx="12" cy="12" r="9"/><path d="M12 7v5l3.5 2"/></svg>',
        "arrow": f'<svg {common}><path d="M4 12h15M13 6l6 6-6 6"/></svg>',
        "shield": f'<svg {common}><path d="M12 3l7 3v6c0 4.5-3 7.5-7 9-4-1.5-7-4.5-7-9V6l7-3z"/><path d="M9 12l2 2 4-4"/></svg>',
        "cpu": f'<svg {common}><rect x="7" y="7" width="10" height="10" rx="1"/><path d="M9 3v3M15 3v3M9 18v3M15 18v3M3 9h3M3 15h3M18 9h3M18 15h3"/></svg>',
        "radar": f'<svg {common}><circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="5"/><circle cx="12" cy="12" r="1.2" fill="{color}"/><path d="M12 12L18 7"/></svg>',
        "gauge": f'<svg {common}><path d="M4 15a8 8 0 1 1 16 0"/><path d="M12 15l4-5"/><circle cx="12" cy="15" r="1.2" fill="{color}"/></svg>',
        "camera": f'<svg {common}><rect x="3" y="7" width="18" height="12" rx="2"/><circle cx="12" cy="13" r="3.4"/><path d="M8 7l1.5-2.5h5L16 7"/></svg>',
        "wheel": f'<svg {common}><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="2"/><path d="M12 4v4M12 16v4M4 12h4M16 12h4"/></svg>',
        "layers": f'<svg {common}><path d="M12 3l9 5-9 5-9-5 9-5z"/><path d="M3 13l9 5 9-5"/></svg>',
        "target": f'<svg {common}><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="4"/><circle cx="12" cy="12" r="0.8" fill="{color}"/></svg>',
        "route": f'<svg {common}><circle cx="6" cy="18" r="2"/><circle cx="18" cy="6" r="2"/><path d="M6 16c0-6 4-4 6-7s6-1 6-3"/></svg>',
        "person": f'<svg {common}><circle cx="12" cy="8" r="3.2"/><path d="M5 20c1-4 4-6 7-6s6 2 7 6"/></svg>',
    }
    return icons.get(kind, "")

def wrap(name, title, body_html, static=True):
    script = ""
    if not static:
        script = '<script data-dc-script>\nclass Component extends DCLogic {}\n</script>'
    return f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <script src="./support.js"></script>
</head>
<body>
<x-dc>
<helmet>
  {FONTS}
  <style>{BASE_CSS}</style>
</helmet>
<div class="slide">
{body_html}
  <div class="islab"><span class="islab-dot"></span>ISLAB</div>
</div>
</x-dc>
{script}
</body>
</html>
"""

def foot(page, note=""):
    return f"""  <div class="foot">
    <div>{note}</div>
    <div class="foot-page">{page:02d} / 10</div>
  </div>
"""

def write(name, html):
    p = os.path.join(OUT, f"{name}.dc.html")
    with open(p, "w", encoding="utf-8") as f:
        f.write(html)
    print("wrote", p, len(html), "bytes")
