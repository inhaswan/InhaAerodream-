# a8minicontrol
A8 mini gimbal camera를 제어&통신하기 위한 파이썬 코드 예시

* SIYI A8 mini user manual : https://www.worldronemarket.com/a8-mini-4k-ai-mini-zoom-gimbal-camera-user-manual/

# 환경설정
* 패키지 클론
    ```bash
    https://github.com/yskim04/a8minicontrol.git
    ```
* 이더넷 케이블을 사용하여 카메라와 PC/온보드 컴퓨터를 연결. 현재는 UDP 통신을 사용하여 구현됨
* 제어판 - 네트워크 및 공유 센터 - 어댑터 설정 변경 - 인터넷 프로토콜 버전4(TCP/IPv4) 선택 - ip 주소를 수동으로 설정
  * For example, IP `192.168.144.20`
  * Netmask `255.255.255.0`

# 사용법
* SIYI SDK Protocol Format : a8 mini user manual 에 전체 목록들 확인 가능
* siyi_message.py와 siyi_sdk.py에 카메라 제어 & 스트리밍과 관련된 함수, 클래스들을 파이썬으로 변환하여 작성함
* 코드를 실제로 사용하는 방법은 a8minicontrol/code 폴더에 간단한 예제들 참고
  * Ex) test_lock_mode.py
   ```python
    import sys
    import os
    from time import sleep
  
    current = os.path.dirname(os.path.realpath(__file__))
    parent_directory = os.path.dirname(current)
  
    sys.path.append(parent_directory)

    from siyi_sdk import SIYISDK

    def test():
        cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    
        if not cam.connect():
            print("No connection ")
            exit(1)

        cam.requestLockMode()
        sleep(2)
        print("Current motion mode: ", cam._motionMode_msg.mode)

        cam.disconnect()

    if __name__ == "__main__":
        test()
    ```
   * requestLockMode 를 사용하여 lock mode를 활성화 할 수 있으며, requestLockMode는 siyi_sdk.py에 정의되어 있음.
* 이 모듈을 따로 사용하고 싶다면, `siyi_sdk.py` `siyi_message.py` `utility.py` `crc16_python.py` 스크립트를 해당 코드 디렉토리에 저장.
    ```python
    from siyi_sdk import SIYISDK
    ```
* Example: `test_gimbal_rotation.py` 를 실행할땐,
    ```bash
    cd a8minicontrol/code
    python3 test_gimbal_rotation.py
    ```
# 비디오 스트리밍
현재는 다른 방법 사용중, rtsp 스트리밍 을 위한 참고자료로만 활용
## Requirements
* OpenCV `sudo apt-get install python3-opencv -y`
* imutils `pip install imutils`
* Gstreamer `https://gstreamer.freedesktop.org/documentation/installing/index.html?gi-language=c`
    
    Ubuntu:
    ```bash
    sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y
    ```
## Example
* rtsp 스트리밍 예제 : code/test_rtsp.py 참고
