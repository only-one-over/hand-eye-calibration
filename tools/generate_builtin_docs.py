from html import escape
from pathlib import Path
import os

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    KeepTogether,
    ListFlowable,
    ListItem,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


DOCUMENTS = {
    "hand_eye_quick_start.pdf": {
        "title": "手眼标定程序详细使用说明",
        "subtitle": "Qt6 手眼标定工具 | 适用于 Eye-In-Hand（眼在手）流程",
        "sections": [
            (
                "一、程序用途与坐标约定",
                [
                    "本程序用于计算机器人末端与相机之间的刚体变换。当前正式支持 Eye-In-Hand，也就是相机安装在机器人末端。Eye-To-Hand（眼在外）入口目前保持禁用。",
                    "程序内部统一使用米（m）、弧度（rad）和 Rodrigues 旋转向量。用户可以在参数页输入 degree/rad、mm/m、Euler、RPY 或 Quaternion，程序会在计算前自动标准化。",
                ],
                [
                    "机器人位姿方向：gripper → base，表示末端坐标系到机器人基座坐标系。",
                    "标定板位姿方向：target → camera，表示标定板坐标系到相机坐标系。",
                    "最终输出方向：camera → gripper，表示相机坐标系到机器人末端坐标系。",
                    "不要把机器人示教器中的位姿方向、相机 SDK 的位姿方向和程序输入方向混用。",
                ],
            ),
            (
                "二、推荐操作流程",
                [
                    "程序顶部页面按工作流程排列。第一次使用时建议按以下顺序操作，不要直接跳过参数和数据质量检查。",
                ],
                [
                    "参数：设置标定模式、算法、位姿格式、单位、机器人和相机名称、标定板规格及相机内参。",
                    "相机内参：如果还没有相机内参，选择至少 6 张、推荐 10～30 张不同姿态的棋盘格图片执行自主标定。",
                    "采集：分别上传机器人坐标和标定板图片。两者必须来自同一轮采集，数量、编号和顺序必须一致。",
                    "当前数据：逐组检查机器人位姿、图片路径、target → camera 位姿、角点检测状态和 PnP RMSE。",
                    "标定结果：先计算五种算法，再查看 AX=XB 一致性、Fixed Target 一致性、异常样本、非线性优化和 Bootstrap 结果。",
                    "导出：确认矩阵方向、单位、算法、日期和误差信息无误后，再导出 JSON、YAML、TXT、C++ 或 Python 文件。",
                ],
            ),
            (
                "三、两种数据输入方式",
                [
                    "图片配对模式用于标准棋盘格手眼标定。每一组数据由一份机器人 TCP 位姿和一张对应时刻的标定板图片组成。程序从图片中检测角点，通过 PnP 计算 target → camera。",
                    "手动点基模式用于用户已知固定点的情况。每一组数据只输入 TCP 6D 位姿和相机测得的 XYZ 点，不输入相机 rx、ry、rz。所有相机 XYZ 必须对应同一个固定物理点。",
                ],
                [
                    "图片模式：机器人位姿和图片必须严格一一对应。拍照后再移动机器人，会导致配对错误。",
                    "点基模式：固定点不能在采集过程中移动，且应覆盖不同方向、距离和 TCP 姿态。",
                    "两种模式的数据不要在同一批次中混合使用。",
                ],
            ),
            (
                "四、相机内参与标定板设置",
                [
                    "相机内参包括 3×3 相机矩阵和 5 个畸变参数 k1、k2、p1、p2、k3。没有可靠内参时，PnP 位姿和最终手眼矩阵都可能出现明显误差。",
                    "棋盘格的内角点数量不是黑白方格数量。例如内角点 9×6 时，实际打印方格数量为 10×7。方格尺寸必须填写实际物理尺寸，不能填写图片像素尺寸。",
                ],
                [
                    "推荐相机内参标定图片数量：10～30 张。",
                    "图片应包含不同距离、倾斜角度和画面位置，并覆盖图像中心及四周。",
                    "图片分辨率必须一致，角点检测成功图片至少 6 张。",
                    "使用生成 PDF 打印的标定板时，打印后必须用尺子检查 100 mm 标尺。",
                ],
            ),
            (
                "五、结果判断与导出",
                [
                    "不要只看某一种算法是否返回矩阵。工程上应同时检查旋转 RMSE、平移 RMSE、平均误差、最大误差、异常样本数、Fixed Target 一致性和独立验证结果。",
                    "Pose Quality Score 会从样本数量、旋转幅度、旋转轴分布和空间覆盖四个方面评价采集质量。算法可求解不等于数据达到高精度要求。",
                ],
                [
                    "可计算：至少 5 组，且相对运动没有退化。",
                    "推荐：至少 12 组，最大相对旋转超过 30°，并且存在两组以上独立旋转轴。",
                    "高精度：建议 20～30 组，最大旋转超过 45°，覆盖三轴、近中远距离和整个相机视场。",
                    "导出的 camera → gripper 矩阵必须结合实际坐标约定验证，不能只凭数值大小判断正确。",
                ],
            ),
            (
                "六、常见问题排查",
                [],
                [
                    "机器人坐标和图片数量不一致：重新检查文件选择顺序和 CSV 行数。",
                    "PnP 失败或 RMSE 很大：检查内参、畸变、棋盘格规格、打印尺寸和图片是否存在反光或运动模糊。",
                    "五种算法结果差异很大：增加旋转激励，避免只做平移或绕单一轴旋转。",
                    "Fixed Target 残差很大：检查 target → camera 方向、机器人位姿方向和异常图片。",
                    "复用旧 PDF：生成按钮会自动复用默认目录中同规格文件；如果修改了棋盘格参数，会生成新的文件。",
                ],
            ),
        ],
    },
    "checkerboard_printing.pdf": {
        "title": "标定板生成、打印与尺寸检查说明",
        "subtitle": "必须保持物理尺寸，否则角点坐标和 PnP 结果会失真",
        "sections": [
            (
                "一、如何生成标定板",
                [
                    "进入“标定板 PDF”页面，程序会读取参数页当前 BoardSpec，并动态生成 Chessboard、ChArUco 或 ArUco Grid。标定板图案不是固定图片，而是由当前参数实时生成。",
                ],
                [
                    "先在参数页确认板型、行列数、方格尺寸、marker 尺寸、marker 间距和字典。",
                    "点击“生成 1:1 单页 PDF”时，文件自动保存到文档/HandEyeCalibration/board_pdfs。",
                    "点击“生成 A4 分块 PDF”时，程序按照 100% 实际尺寸切成多页 A4。",
                    "同规格和同输出模式的 PDF 已存在时，生成按钮会直接复用，不重复绘制。",
                    "如需改变保存位置，使用“另存 1:1 单页 PDF”或“另存 A4 分块 PDF”。",
                ],
            ),
            (
                "二、Chessboard 参数含义",
                [
                    "Chessboard 的行列数填写内角点数量，不是黑白格数量。程序自动按照内角点数量加一生成实际方格。",
                ],
                [
                    "内角点 9×6：实际方格为 10×7。",
                    "方格尺寸 25 mm：每个黑格或白格的边长为 25 mm。",
                    "标定板有效尺寸：宽度 =（内角点列数 + 1）× 方格尺寸；高度 =（内角点行数 + 1）× 方格尺寸。",
                    "打印后的外观边缘可能有少量空白，但内部方格尺寸必须保持准确。",
                ],
            ),
            (
                "三、ChArUco 与 ArUco Grid 参数",
                [
                    "ChArUco 同时使用棋盘格方格和 ArUco marker。marker 尺寸必须小于方格尺寸，字典必须与检测设置一致。",
                    "ArUco Grid 由 marker 行列、marker 尺寸和 marker 间距决定。marker separation 是相邻 marker 之间的实际间隔，不再由程序固定推算。",
                ],
                [
                    "更换字典后必须重新生成并重新采集图片，不能沿用旧图案。",
                    "修改 marker 尺寸或间距后，旧 PDF 不属于同规格文件，程序会生成新的 PDF。",
                    "打印 ArUco 图案时不要压缩或拉伸单个页面，二维码边缘必须清晰。",
                ],
            ),
            (
                "四、1:1 打印要求",
                [
                    "打印对话框必须选择实际大小或 100%。禁止选择“适应页面”“缩小到可打印区域”“适合纸张”或自动缩放。",
                    "A4 分块版本用于普通打印机拼接。每一页包含边界和 100 mm 标尺，不应使用打印机的无边距缩放功能改变图案比例。",
                ],
                [
                    "打印后用钢尺测量 100 mm 标尺，建议误差不超过 0.5 mm。",
                    "拼接前先确认各页方向、边界和重叠区域，再用胶带从背面固定。",
                    "标定板应保持平整，不要折叠、起皱或贴在明显弯曲的表面。",
                    "如果使用玻璃或覆膜板，应避免强反光，必要时调整光源和相机角度。",
                ],
            ),
            (
                "五、尺寸错误的影响",
                [
                    "标定板尺寸错误会直接影响 PnP 的平移尺度。即使角点检测成功，最终平移矩阵也可能按比例错误。",
                    "如果打印后 100 mm 标尺实际只有 98 mm，程序仍会按照参数中的理论尺寸计算，因此必须先修正打印缩放或重新生成文件。",
                ],
                [
                    "不要把截图、Word 文档或图片查看器中的缩放图直接当作标定板打印。",
                    "不要在打印后再次使用扫描、拍照或图像编辑软件改变尺寸。",
                    "如果标定板局部损坏、缺角、反光严重或 marker 模糊，应重新制作。",
                ],
            ),
        ],
    },
    "data_collection_quality.pdf": {
        "title": "手眼标定数据采集质量与验收说明",
        "subtitle": "高质量标定依赖充分运动激励和可靠的一一对应数据",
        "sections": [
            (
                "一、样本数量等级",
                [
                    "样本数量只是最低条件，不能单独代表标定精度。程序会结合运动激励、空间覆盖、PnP 误差和固定目标一致性综合判断。",
                ],
                [
                    "少于 5 组：不能通过可靠性流水线。",
                    "5～11 组：通常只能达到可计算等级。",
                    "12～19 组：达到推荐采集规模。",
                    "20～30 组：适合工程高精度标定，但仍需覆盖多轴、多距离和全视场。",
                ],
            ),
            (
                "二、机器人运动激励",
                [
                    "手眼标定依赖不同姿态之间的相对运动。仅改变 XYZ 平移、只绕一根轴旋转或所有姿态变化很小，都会导致 AX=XB 问题接近退化。",
                ],
                [
                    "让 TCP 绕 X、Y、Z 三个方向产生明显不同的旋转。",
                    "最大相对旋转建议超过 30°，高精度建议超过 45°。",
                    "避免所有姿态都集中在一个很小的角度范围内。",
                    "工作距离应覆盖近、中、远至少三个层次。",
                    "机器人末端移动时避免急停和振动，拍照前等待机械臂稳定。",
                ],
            ),
            (
                "三、相机视场与标定板位置",
                [
                    "棋盘格不能始终位于图像中心。应让标定板在图像中心、左上、右上、左下、右下和边缘区域出现，并改变倾斜角度。",
                ],
                [
                    "中心位置：用于获得稳定的基础角点。",
                    "四周位置：用于覆盖镜头畸变和整个 FOV。",
                    "不同倾斜角：用于提高深度和旋转估计的可观测性。",
                    "不同距离：用于检查平移尺度和工作空间一致性。",
                ],
            ),
            (
                "四、图片与 PnP 质量检查",
                [
                    "每张图片都必须能清楚看到足够的棋盘格角点或 marker。程序优先使用 findChessboardCornersSB，必要时回退到传统检测，并记录实际检测方式。",
                ],
                [
                    "避免运动模糊、过曝、欠曝、强反光、遮挡和棋盘格出界。",
                    "确认图片分辨率一致，机器人坐标和图片顺序一一对应。",
                    "检查每组 PnP RMSE，明显偏大的样本应回看原图。",
                    "不要因为某种算法返回矩阵就忽略图片检测失败和异常样本。",
                ],
            ),
            (
                "五、可靠性流水线",
                [
                    "推荐使用结果页的完整可靠性流水线。程序依次执行运动激励检查、PnP 质量检查、五算法计算、AX=XB 一致性、Fixed Target 一致性、异常样本验证、归一化 Huber 优化和 Bootstrap 重采样。",
                ],
                [
                    "异常样本不会只按单个 RMSE 直接删除，而是逐个删除并重新计算，只有误差下降且剩余数据仍可求解时才正式剔除。",
                    "归一化 Huber 同时考虑旋转和位移残差，默认按 1° 和 1 mm 归一化。",
                    "Bootstrap 会重复执行原始算法和非线性精修，并输出 successRate、置信区间和不确定度。",
                    "最终矩阵应同时参考优化前后误差、固定目标误差和独立验证数据。",
                ],
            ),
            (
                "六、采集完成前检查清单",
                [],
                [
                    "样本数量达到目标，且没有重复或错误编号。",
                    "机器人位姿与图片数量、顺序完全一致。",
                    "相机内参和畸变参数来自相同分辨率。",
                    "相对旋转覆盖多轴，工作距离和图像位置有变化。",
                    "所有图片可以追溯到具体样本，异常图片已经单独记录。",
                    "独立验证数据没有参与训练，且验证结果达到项目阈值。",
                ],
            ),
        ],
    },
}


def find_font():
    windows_fonts = Path(os.environ.get("WINDIR", r"C:\Windows")) / "Fonts"
    candidates = [
        windows_fonts / "simhei.ttf",
        windows_fonts / "Noto Sans SC (TrueType).otf",
        windows_fonts / "Deng.ttf",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RuntimeError("未找到可嵌入的中文字体，请安装 SimHei、Noto Sans SC 或等效中文字体。")


def register_fonts():
    font_path = find_font()
    pdfmetrics.registerFont(TTFont("DocChinese", str(font_path)))
    return "DocChinese"


def paragraph(text, style):
    return Paragraph(escape(text).replace("\n", "<br/>") , style)


def bullets(items, style):
    return ListFlowable(
        [ListItem(paragraph(item, style), leftIndent=4) for item in items],
        bulletType="bullet",
        start="circle",
        leftIndent=18,
        bulletFontName=style.fontName,
        bulletFontSize=8,
    )


def page_header_footer(canvas, document, font_name):
    canvas.saveState()
    width, height = A4
    canvas.setStrokeColor(colors.HexColor("#D5DCE5"))
    canvas.setLineWidth(0.5)
    canvas.line(18 * mm, height - 14 * mm, width - 18 * mm, height - 14 * mm)
    canvas.setFont(font_name, 8)
    canvas.setFillColor(colors.HexColor("#667085"))
    canvas.drawString(18 * mm, height - 10 * mm, "Qt6 手眼标定程序")
    canvas.drawRightString(width - 18 * mm, 10 * mm, f"第 {document.page} 页")
    canvas.restoreState()


def build_pdf(path, document_data, font_name):
    styles = getSampleStyleSheet()
    title_style = ParagraphStyle(
        "DocTitle", parent=styles["Title"], fontName=font_name, fontSize=22,
        leading=30, alignment=TA_CENTER, textColor=colors.HexColor("#123B63"),
        spaceAfter=8 * mm,
    )
    subtitle_style = ParagraphStyle(
        "DocSubtitle", parent=styles["Normal"], fontName=font_name, fontSize=10,
        leading=16, alignment=TA_CENTER, textColor=colors.HexColor("#667085"),
        spaceAfter=10 * mm,
    )
    heading_style = ParagraphStyle(
        "DocHeading", parent=styles["Heading2"], fontName=font_name, fontSize=15,
        leading=22, textColor=colors.HexColor("#123B63"), spaceBefore=6 * mm,
        spaceAfter=3 * mm,
    )
    body_style = ParagraphStyle(
        "DocBody", parent=styles["BodyText"], fontName=font_name, fontSize=10.5,
        leading=18, textColor=colors.HexColor("#273444"), spaceAfter=3 * mm,
    )
    bullet_style = ParagraphStyle(
        "DocBullet", parent=body_style, leftIndent=4 * mm, firstLineIndent=0,
        spaceAfter=1.5 * mm,
    )
    note_style = ParagraphStyle(
        "DocNote", parent=body_style, fontSize=9, leading=15,
        textColor=colors.HexColor("#475467"), backColor=colors.HexColor("#F2F6FA"),
        borderColor=colors.HexColor("#D5E3F0"), borderWidth=0.5,
        borderPadding=5 * mm, spaceBefore=3 * mm, spaceAfter=5 * mm,
    )

    doc = SimpleDocTemplate(
        str(path), pagesize=A4, rightMargin=18 * mm, leftMargin=18 * mm,
        topMargin=21 * mm, bottomMargin=18 * mm, title=document_data["title"],
        author="Qt6 手眼标定程序",
    )
    story = [
        Spacer(1, 7 * mm),
        paragraph(document_data["title"], title_style),
        paragraph(document_data["subtitle"], subtitle_style),
        paragraph(
            "本文档为程序内置说明。实际使用时请结合当前参数页的板型、单位、相机内参和输入方向进行确认。",
            note_style,
        ),
    ]

    for index, (heading, paragraphs, bullet_items) in enumerate(document_data["sections"]):
        story.append(Paragraph(escape(heading), heading_style))
        for text in paragraphs:
            story.append(paragraph(text, body_style))
        if bullet_items:
            story.append(bullets(bullet_items, bullet_style))
        if index < len(document_data["sections"]) - 1:
            story.append(Spacer(1, 2 * mm))

    story.append(Spacer(1, 6 * mm))
    story.append(paragraph("说明：程序会优先使用程序目录下 docs/ 中的同名外置文档；缺失时回退到内置版本。", note_style))
    doc.build(
        story,
        onFirstPage=lambda canvas, doc: page_header_footer(canvas, doc, font_name),
        onLaterPages=lambda canvas, doc: page_header_footer(canvas, doc, font_name),
    )


def main():
    target = Path(__file__).resolve().parents[1] / "src" / "assets" / "docs"
    target.mkdir(parents=True, exist_ok=True)
    font_name = register_fonts()
    for name, document_data in DOCUMENTS.items():
        build_pdf(target / name, document_data, font_name)


if __name__ == "__main__":
    main()
