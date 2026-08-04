#!/usr/bin/env python3
from pathlib import Path
from PIL import Image, ImageChops


ROOT = Path(__file__).resolve().parents[1]
IMG = ROOT / "deliverables" / "report_images"


def trim_white(source, target, border=24):
    image = Image.open(source).convert("RGB")
    background = Image.new("RGB", image.size, "white")
    diff = ImageChops.difference(image, background).convert("L")
    bbox = diff.point(lambda p: 255 if p > 12 else 0).getbbox()
    if bbox:
        left, top, right, bottom = bbox
        left = max(0, left - border)
        top = max(0, top - border)
        right = min(image.width, right + border)
        bottom = min(image.height, bottom + border)
        image = image.crop((left, top, right, bottom))
    image.save(target, optimize=True)


def crop(source, target, box):
    image = Image.open(source).convert("RGB")
    image.crop(box).save(target, optimize=True)


trim_white(IMG / "piab_u10.jpg", IMG / "fig_piab_u10.png", 35)
trim_white(IMG / "smc_zp2_thin_flat.jpg", IMG / "fig_smc_zp2_thin_flat.png", 12)
trim_white(IMG / "smc_zp3p_film.jpg", IMG / "fig_smc_zp3p_film.png", 12)
trim_white(IMG / "schmalz_sff_sfb1_product.png", IMG / "fig_schmalz_sff_sfb1_product.png", 12)

crop(
    IMG / "dexnet3_page1.png",
    IMG / "fig_dexnet_seal_model.png",
    (505, 220, 946, 750),
)
crop(
    IMG / "dexnet3_page3.png",
    IMG / "fig_dexnet_wrench_model.png",
    (505, 10, 970, 475),
)
crop(
    IMG / "hybrid_page3.png",
    IMG / "fig_hybrid_suction_then_grasp.png",
    (75, 790, 945, 1085),
)

print("Prepared:")
for path in sorted(IMG.glob("fig_*.png")):
    print(path.name, Image.open(path).size)
