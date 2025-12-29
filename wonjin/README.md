# 25-1 컴퓨터 그래픽스 실습 - 중간고사

## 디렉터리 구조

```
MidtermExam_홍원진/
├── VTK_Mocap.py            # 메인 스크립트
└── src/
    ├── data.py             # 네트워크 스레드, 키 입력 이벤트 콜백
    ├── callback.py         # TimerCallback 클래스
    └── utils.py            # vtk 관련 함수, 쿼터니언 계산 관련 함수
```

## 실행 방법

1. vtk 설치
```bash
pip install vtk
```


2. 실행
```bash
python VTK_main.py
```


- key 입력 ( in `VTK_main.py`)
  * `r`: 데이터 녹화 시작
  * `s`: 데이터 녹화 종료 및 `Recorded_Data/sensor_data_<timestamp>.csv`로 저장
  * `c`: 현재 쿼터니언을 초기화 (캘리브레이션)


<br>


3. 실행 후, 녹화한 데이터 불러오기
```bash
python read_csv.py --path {your_path}.csv
```