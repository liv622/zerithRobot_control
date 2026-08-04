#!/usr/bin/env python3
from pathlib import Path
import math
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "deliverables" / "report_images"
OUT.mkdir(parents=True, exist_ok=True)

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size)


NAVY = "#173B57"
TEAL = "#176B87"
CYAN = "#67B7C8"
ORANGE = "#E07A5F"
GREEN = "#3D8B74"
PALE = "#EDF5F7"
GRID = "#C8D5DA"
TEXT = "#243042"
RED = "#C94B54"
YELLOW = "#E9A23B"


def title(draw, text, subtitle=None):
    draw.text((70, 42), text, font=font(40, True), fill=NAVY)
    if subtitle:
        draw.text((72, 100), subtitle, font=font(22), fill="#526772")


def rounded(draw, xy, fill="white", outline=GRID, radius=20, width=3):
    draw.rounded_rectangle(xy, radius=radius, fill=fill, outline=outline, width=width)


def fig_modules():
    im = Image.new("RGB", (1800, 1120), "white")
    d = ImageDraw.Draw(im)
    title(d, "六种小型吸盘模块概念", "从狭小吸点、通用吸取到粗糙/透气表面的分层配置")
    cards = [
        ("A  单杯 φ8", "最小体积 / 小吸点", "目录力 3.0 N @ −60 kPa"),
        ("B  单杯 φ13", "通用主力 / 较大余量", "目录力 8.0 N @ −60 kPa"),
        ("C  双杯 2×φ10", "分散载荷 / 抗俯仰", "合计 9.4 N @ −60 kPa"),
        ("D  四杯 4×φ6", "局部顺应 / 冗余密封", "合计 6.8 N @ −60 kPa"),
        ("E  椭圆 4×20", "窄长吸附面 / 条状商品", "几何估算 3.77 N @ −60 kPa"),
        ("F  海绵/泡棉", "粗糙、纹理、有限透气", "按实测压力×有效面积定级"),
    ]
    for i, (name, use, force) in enumerate(cards):
        col, row = i % 3, i // 3
        x, y = 65 + col * 575, 165 + row * 440
        rounded(d, (x, y, x + 525, y + 380), PALE if i < 5 else "#FFF4E8")
        d.text((x + 28, y + 22), name, font=font(29, True), fill=TEAL)
        cx, cy = x + 262, y + 175
        d.rectangle((cx - 100, y + 82, cx + 100, y + 105), fill=NAVY)
        if i == 0:
            d.ellipse((cx - 48, cy - 48, cx + 48, cy + 48), outline=TEAL, width=12)
        elif i == 1:
            d.ellipse((cx - 73, cy - 73, cx + 73, cy + 73), outline=TEAL, width=14)
        elif i == 2:
            for off in (-85, 85):
                d.ellipse((cx + off - 55, cy - 55, cx + off + 55, cy + 55), outline=TEAL, width=12)
        elif i == 3:
            for ox in (-66, 66):
                for oy in (-55, 55):
                    d.ellipse((cx + ox - 36, cy + oy - 36, cx + ox + 36, cy + oy + 36), outline=TEAL, width=10)
        elif i == 4:
            d.ellipse((cx - 125, cy - 38, cx + 125, cy + 38), outline=TEAL, width=12)
        else:
            d.rounded_rectangle((cx - 135, cy - 58, cx + 135, cy + 58), radius=26, fill="#607D88")
            for px in range(cx - 105, cx + 106, 35):
                d.ellipse((px - 6, cy - 6, px + 6, cy + 6), fill="white")
        d.text((x + 28, y + 265), use, font=font(23, True), fill=TEXT)
        d.text((x + 28, y + 315), force, font=font(21), fill="#536772")
    d.text((70, 1050), "注：A–D 为 SMC ZP2 目录理论力的组合；E 为椭圆面积估算；F 不允许仅按封闭面积外推。", font=font(20), fill="#65767E")
    im.save(OUT / "fig_solution_modules.png")


def fig_force():
    im = Image.new("RGB", (1800, 1120), "white")
    d = ImageDraw.Draw(im)
    title(d, "吸力不是载荷：六种方案的三级折减", "−60 kPa；平整致密表面；“项目建议载荷”含动态、密封与安全折减")
    labels = ["φ8 单杯", "φ13 单杯", "2×φ10", "4×φ6", "4×20 椭圆*", "φ20 单杯"]
    forces = [3.0, 8.0, 9.4, 6.8, 3.77, 18.0]
    top_g = [61, 164, 192, 139, 77, 368]
    side_g = [31, 82, 96, 70, 39, 184]
    max_force = 20
    left, top, right, bottom = 245, 190, 1690, 940
    for tick in range(0, 21, 5):
        x = left + (right - left) * tick / max_force
        d.line((x, top, x, bottom), fill=GRID, width=2)
        d.text((x - 16, bottom + 15), str(tick), font=font(20), fill="#5F7078")
    for i, label in enumerate(labels):
        y = top + 45 + i * 116
        d.text((45, y - 19), label, font=font(23, True), fill=TEXT)
        width = (right - left) * forces[i] / max_force
        d.rounded_rectangle((left, y - 26, left + width, y + 26), radius=12, fill=TEAL)
        d.text((left + width + 12, y - 20), f"{forces[i]:g} N", font=font(22, True), fill=TEAL)
        d.text((left + 15, y + 37), f"顶部建议 {top_g[i]} g ｜ 侧向建议 {side_g[i]} g", font=font(19), fill="#536772")
    d.text((720, 1015), "目录/理论吸力", font=font(22, True), fill=TEAL)
    d.text((70, 1060), "* 椭圆杯按 4×20 mm 椭圆几何面积估算，实际必须以厂商有效面积或拉脱试验替换。", font=font(19), fill="#65767E")
    im.save(OUT / "fig_force_comparison.png")


def fig_mounts():
    im = Image.new("RGB", (1800, 1120), "white")
    d = ImageDraw.Draw(im)
    title(d, "吸盘安装在夹爪上的四种可制造架构", "中心伸缩优先；侧挂适合改造；指尖双杯适合定制；翻转舱适合多模式")
    names = [
        ("① 掌部中央伸缩", "同轴、碰撞包络小", "推荐默认"),
        ("② 法兰夹层侧挂", "不改夹爪本体、改造快", "推荐改造"),
        ("③ 指尖双杯集成", "杯距可调、先吸后夹", "定制方案"),
        ("④ 90°翻转吸盘舱", "收纳彻底、模式切换", "复杂方案"),
    ]
    for i, (name, desc, tag) in enumerate(names):
        col, row = i % 2, i // 2
        x, y = 75 + col * 865, 165 + row * 445
        rounded(d, (x, y, x + 790, y + 385), "white")
        d.text((x + 28, y + 24), name, font=font(29, True), fill=TEAL)
        d.rounded_rectangle((x + 600, y + 25, x + 755, y + 70), radius=20, fill=GREEN if i < 2 else ORANGE)
        d.text((x + 622, y + 32), tag, font=font(19, True), fill="white")
        cx, cy = x + 395, y + 205
        d.rectangle((cx - 135, y + 95, cx + 135, y + 135), fill=NAVY)
        d.rectangle((cx - 190, y + 135, cx - 115, y + 300), fill="#758D98")
        d.rectangle((cx + 115, y + 135, cx + 190, y + 300), fill="#758D98")
        if i == 0:
            d.line((cx, y + 135, cx, y + 250), fill=ORANGE, width=18)
            d.ellipse((cx - 45, y + 235, cx + 45, y + 295), outline=ORANGE, width=12)
        elif i == 1:
            d.line((cx + 135, y + 115, cx + 275, y + 115), fill=ORANGE, width=16)
            d.line((cx + 275, y + 115, cx + 275, y + 255), fill=ORANGE, width=16)
            d.ellipse((cx + 230, y + 240, cx + 320, y + 300), outline=ORANGE, width=12)
        elif i == 2:
            for px in (cx - 153, cx + 153):
                d.ellipse((px - 38, y + 270, px + 38, y + 322), outline=ORANGE, width=10)
        else:
            d.arc((cx - 110, y + 110, cx + 110, y + 300), 260, 70, fill=ORANGE, width=12)
            d.line((cx, y + 135, cx + 145, y + 210), fill=ORANGE, width=17)
            d.ellipse((cx + 105, y + 195, cx + 190, y + 255), outline=ORANGE, width=11)
        d.text((x + 28, y + 335), desc, font=font(22), fill="#536772")
    d.text((70, 1060), "设计控制量：伸出量、回缩避让、吸盘 TCP 偏置、软管最小弯曲半径、附加质量与绕法兰惯量。", font=font(20), fill="#65767E")
    im.save(OUT / "fig_mounting_architectures.png")


def fig_risk():
    im = Image.new("RGB", (1800, 1180), "white")
    d = ImageDraw.Draw(im)
    title(d, "商品特性—吸附风险矩阵", "风险来自局部密封、泄漏、变形、摩擦和载荷路径，而不是“物体名称”本身")
    cols = ["密封", "泄漏", "形变/损伤", "侧滑", "多取/错取"]
    rows = [
        ("平整致密、无孔", [0, 0, 1, 1, 0]),
        ("浅纹理/覆膜接缝", [1, 1, 1, 1, 0]),
        ("粗糙纸板/木纹", [2, 2, 1, 1, 0]),
        ("软袋、薄膜、柔性片", [1, 1, 2, 2, 2]),
        ("曲面/锥面/异形", [2, 1, 1, 2, 0]),
        ("湿润、油污、冷凝", [1, 1, 1, 2, 0]),
        ("网孔、织物、开孔泡棉", [2, 2, 1, 2, 1]),
        ("吸点远离重心", [0, 0, 1, 2, 0]),
    ]
    x0, y0 = 470, 205
    cw, rh = 245, 96
    for j, c in enumerate(cols):
        d.rectangle((x0 + j * cw, y0, x0 + (j + 1) * cw, y0 + rh), fill=TEAL, outline="white", width=3)
        d.text((x0 + j * cw + 68, y0 + 28), c, font=font(24, True), fill="white")
    colors = ["#D9F0E7", "#FFE9B8", "#F5C3C7"]
    words = ["低", "中", "高"]
    for i, (name, vals) in enumerate(rows):
        y = y0 + (i + 1) * rh
        d.rectangle((65, y, x0, y + rh), fill=PALE if i % 2 == 0 else "white", outline=GRID, width=2)
        d.text((90, y + 30), name, font=font(22, True), fill=TEXT)
        for j, val in enumerate(vals):
            x = x0 + j * cw
            d.rectangle((x, y, x + cw, y + rh), fill=colors[val], outline="white", width=3)
            d.text((x + 104, y + 30), words[val], font=font(23, True), fill=[GREEN, "#A66A00", RED][val])
    d.text((70, 1085), "高风险并非自动禁用：需要更换杯型/真空源、改变动作（吸提后夹）或转入专项实测；网孔跨越密封唇通常直接判退。", font=font(20), fill="#65767E")
    im.save(OUT / "fig_risk_matrix.png")


def fig_retail_extremes():
    im = Image.new("RGB", (1800, 1100), "white")
    d = ImageDraw.Draw(im)
    title(d, "商超零售的两个极端任务族", "同一套吸夹末端，在“制造间隙”和“最终搬运”时承担的物理责任完全不同")
    panels = [
        (70, "① 扁平 / 紧密堆叠", "吸盘先制造夹爪入口", "#EAF6F8"),
        (920, "② 光滑刚性商品", "吸盘承担密封与动态搬运", "#FFF4E8"),
    ]
    for x, name, desc, fill in panels:
        rounded(d, (x, 165, x + 810, 965), fill=fill, outline=CYAN if x < 500 else ORANGE)
        d.text((x + 35, 195), name, font=font(34, True), fill=TEAL)
        d.text((x + 35, 250), desc, font=font(23), fill="#536772")
    # Tight stack: shelf, stack, suction, gripper gap.
    x = 70
    d.rectangle((x + 90, 760, x + 720, 815), fill="#687D87")
    for k in range(5):
        y = 735 - k * 62
        d.rounded_rectangle((x + 165, y, x + 650, y + 48), radius=10, fill="#D8A35D", outline="#986B2F", width=3)
    d.rectangle((x + 355, 315, x + 460, 360), fill=NAVY)
    d.line((x + 407, 360, x + 407, 470), fill=ORANGE, width=18)
    d.ellipse((x + 362, 455, x + 452, 510), outline=ORANGE, width=12)
    d.line((x + 650, 690, x + 650, 600), fill=GREEN, width=8)
    d.polygon([(x + 650, 580), (x + 632, 612), (x + 668, 612)], fill=GREEN)
    d.text((x + 675, 615), "先提起 10–40 mm", font=font(21, True), fill=GREEN)
    d.text((x + 115, 855), "极端项：零侧隙｜靠壁｜层间粘连｜多取｜大悬垂", font=font(21, True), fill=TEXT)
    d.text((x + 115, 905), "能力判据：F吸 ≥ 动态重量 + 分离力；夹爪能否插入", font=font(20), fill="#536772")
    # Rigid objects: jar, can, tub and forces.
    x = 920
    # jar
    d.rounded_rectangle((x + 85, 485, x + 285, 760), radius=48, fill="#B8DBE8", outline=TEAL, width=5)
    d.rectangle((x + 110, 435, x + 260, 500), fill="#738B96")
    # can
    d.ellipse((x + 340, 450, x + 540, 520), fill="#BFC8CC", outline="#65757C", width=4)
    d.rectangle((x + 340, 485, x + 540, 760), fill="#DCE2E5", outline="#65757C", width=4)
    d.ellipse((x + 340, 725, x + 540, 790), fill="#BFC8CC", outline="#65757C", width=4)
    # tub
    d.polygon([(x + 590, 505), (x + 755, 505), (x + 720, 760), (x + 625, 760)], fill="#E4C98F", outline="#9B762A")
    d.rectangle((x + 575, 470, x + 770, 525), fill="#8DBA78")
    # cups
    for cx, cy in [(x + 185, 410), (x + 440, 425), (x + 673, 450)]:
        d.rectangle((cx - 55, cy - 75, cx + 55, cy - 52), fill=NAVY)
        d.ellipse((cx - 47, cy - 60, cx + 47, cy), outline=ORANGE, width=11)
    d.text((x + 95, 825), "极端项：高质量｜小平面｜大曲率｜重心偏置｜冷凝湿滑", font=font(21, True), fill=TEXT)
    d.text((x + 95, 875), "能力判据：杯径/真空/曲率/力矩/加速度全通过", font=font(20), fill="#536772")
    d.text((70, 1025), "决策原则：第一类优先“短程吸提后夹”；第二类只有通过整程动态验证才允许纯吸运输。", font=font(22, True), fill=NAVY)
    im.save(OUT / "fig_retail_extreme_scenarios.png")


def fig_rigid_boundary():
    im = Image.new("RGB", (1800, 1120), "white")
    d = ImageDraw.Draw(im)
    title(d, "光滑刚性商品：质量—杯径初始筛选边界", "采用 η=0.8；SMC 顶吸系数4、侧吸系数8；仅用于平整气密面初筛")
    left, top, right, bottom = 190, 170, 1640, 920
    xmax, ymax = 600, 42
    for mass in range(0, 601, 100):
        x = left + (right - left) * mass / xmax
        d.line((x, top, x, bottom), fill=GRID, width=2)
        d.text((x - 18, bottom + 18), str(mass), font=font(19), fill="#5F7078")
    for dia in range(0, 43, 5):
        y = bottom - (bottom - top) * dia / ymax
        d.line((left, y, right, y), fill=GRID, width=2)
        d.text((125, y - 13), str(dia), font=font(19), fill="#5F7078")
    d.text((770, 1010), "商品质量 / g", font=font(23, True), fill=TEXT)
    d.text((35, 490), "所需等效圆杯直径 / mm", font=font(21, True), fill=TEXT)
    curves = [
        ("−60 kPa 顶吸", 60000, 4, TEAL),
        ("−40 kPa 顶吸", 40000, 4, GREEN),
        ("−60 kPa 侧吸", 60000, 8, ORANGE),
        ("−40 kPa 侧吸", 40000, 8, RED),
    ]
    for idx, (name, pressure, safety, color) in enumerate(curves):
        pts = []
        for mass_g in range(1, xmax + 1, 3):
            force = (mass_g / 1000) * 9.81 * safety / 0.8
            diameter_m = math.sqrt(4 * force / (math.pi * pressure))
            dia_mm = diameter_m * 1000
            x = left + (right - left) * mass_g / xmax
            y = bottom - (bottom - top) * dia_mm / ymax
            pts.append((x, y))
        d.line(pts, fill=color, width=7)
        lx, ly = 1030 + (idx % 2) * 300, 190 + (idx // 2) * 55
        d.line((lx, ly, lx + 55, ly), fill=color, width=7)
        d.text((lx + 65, ly - 16), name, font=font(20, True), fill=color)
    for dia in (8, 10, 13, 16, 20):
        y = bottom - (bottom - top) * dia / ymax
        d.line((left, y, right, y), fill="#8B9BA2", width=2)
        d.text((right + 12, y - 12), f"φ{dia}", font=font(18, True), fill="#64757C")
    d.text((70, 1060), "警告：曲面、杯唇跨缝、湿滑、重心偏置或盖体松动时，本图失效；须换真实目录力并做拉脱、侧滑和动态试验。", font=font(20), fill=RED)
    im.save(OUT / "fig_rigid_mass_diameter_boundary.png")


if __name__ == "__main__":
    fig_modules()
    fig_force()
    fig_mounts()
    fig_risk()
    fig_retail_extremes()
    fig_rigid_boundary()
    print("generated", *(p.name for p in sorted(OUT.glob("fig_*"))), sep="\n")
