import base64
import urllib
import requests
import json
import cv2  # 用于访问摄像头
import os  # 用于文件操作
#百度云的API服务
API_KEY = ""
SECRET_KEY = ""
PHOTO_PATH = "captured_face.jpg"  # 拍摄的照片保存路径

# 添加用户ID与人名的映射字典
USER_ID_TO_NAME = {
    "1": "卓",
    "2": "徐总",
    "3": "张主管",
    "4": "赵工",
    # 可以根据需要继续添加更多用户ID和对应的人名
}


def capture_photo():
    """调用摄像头拍摄一张照片并保存"""
    # 打开摄像头（0表示默认摄像头）
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("无法打开摄像头")
        return False

    # 拍摄一帧
    ret, frame = cap.read()

    if ret:
        # 保存照片
        cv2.imwrite(PHOTO_PATH, frame)
        # print(f"照片已保存至: {PHOTO_PATH}")
    else:
        print("拍摄失败")
        return False

    # 释放摄像头资源
    cap.release()
    return True


def main():
    # 先调用摄像头拍摄照片
    if not capture_photo():
        print("拍摄照片失败，程序退出")
        return

    # 检查照片是否存在
    if not os.path.exists(PHOTO_PATH):
        print(f"照片文件不存在: {PHOTO_PATH}")
        return

    url = "https://aip.baidubce.com/rest/2.0/face/v3/search?access_token=" + get_access_token()

    # 获取拍摄照片的base64编码
    image_base64 = get_file_content_as_base64(PHOTO_PATH, False)

    payload = json.dumps({
        "group_id_list": "1",
        "image": image_base64,  # 使用拍摄的照片
        "image_type": "BASE64"
    }, ensure_ascii=False)

    # 添加请求头
    headers = {
        'Content-Type': 'application/json',
        'Accept': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload.encode("utf-8"))

    # print("识别结果：")
    # print(response.text)

    # 解析响应并输出对应的人名
    try:
        result = json.loads(response.text)
        if result.get("error_code") == 0:
            # 获取识别到的user_id
            user_id = result["result"]["user_list"][0]["user_id"]
            # 获取对应的人名，如果没有则显示未知
            user_name = USER_ID_TO_NAME.get(user_id, f"未知用户（ID: {user_id}）")
            print(f"这个人是：{user_name}，向他问好")
        else:
            print(f"识别失败: {result.get('error_msg')}")
    except Exception as e:
        print(f"解析识别结果失败: {e}")


def get_file_content_as_base64(path, urlencoded=False):
    """获取文件base64编码"""
    with open(path, "rb") as f:
        content = base64.b64encode(f.read()).decode("utf8")
        if urlencoded:
            content = urllib.parse.quote_plus(content)
    return content


def get_access_token():
    """使用 AK，SK 生成鉴权签名（Access Token）"""
    url = "https://aip.baidubce.com/oauth/2.0/token"
    params = {"grant_type": "client_credentials", "client_id": API_KEY, "client_secret": SECRET_KEY}
    try:
        response = requests.post(url, params=params)
        return str(response.json().get("access_token"))
    except Exception as e:
        print(f"获取access_token失败: {e}")
        return None


if __name__ == '__main__':
    main()
