import subprocess
import logging
from src.utils.logging_config import get_logger

logger = get_logger(__name__)

async def identify_function(args: dict) -> str:
    """
    响应“我是谁”“认识我”指令，调用111.py脚本执行身份识别
    :param args: 工具调用参数（无参数）
    :return: 执行结果文本
    """
    try:
        # 指定Python解释器和脚本路径（与需求一致）
        python_path = r"/home/zhuo/miniconda3/bin/python3"
        script_path = r"/home/zhuo/PycharmProjects/PythonProject/face/face.py"

        # 执行脚本（使用虚拟环境Python）
        result = subprocess.run(
            [python_path, script_path],
            capture_output=True,
            text=True,
            check=True,
            encoding="utf-8",
            errors="ignore"
        )

        # 打印并记录输出
        print(f"[{script_path} 输出] {result.stdout.strip()}")
        logger.info(f"身份识别脚本执行输出: {result.stdout.strip()}")

        # 构造返回结果
        output = result.stdout.strip() or "身份识别操作执行成功（无输出）"
        return f"已执行身份识别：{output}"

    except subprocess.CalledProcessError as e:
        error_msg = f"脚本执行失败（状态码：{e.returncode}）：{e.stderr.strip()}"
        logger.error(error_msg)
        return error_msg
    except FileNotFoundError:
        error_msg = f"未找到文件，请检查路径：Python={python_path}，脚本={script_path}"
        logger.error(error_msg)
        return error_msg
    except Exception as e:
        error_msg = f"身份识别工具异常：{str(e)}"
        logger.error(error_msg)
        return error_msg
