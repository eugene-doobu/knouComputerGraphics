# 2차원 뷰잉

뷰잉이란 어떠한 장면을 눈으로 보는 것처럼 여러 가지 물체로 구성된 장면의 일정 영역을 화면상의 정해진 영역에 그리는 과정을 의미

2차원 뷰잉의 경우 월드좌표의 직사각형 영역을 지정하며, 그 영역의 좌표를 화면에 표시하기 위한 스크린 좌표로 변환하는 기하변환을 통해 정의됨

- 클리핑: 화면에 디스플레이할 장면의 일부 영역을 지정하여 그 영역을 벗어나는 물체를 잘라내는 처리
- 뷰포트: 출력장치에서 클리핑 윈도 내부의 영역이 표시될 영역

## 2차원 뷰잉 파이프라인

### 1. 2차원 뷰잉의 처리 흐름

1. 모델링 좌표: 개별 객체를 설계하는 좌표
1. 세계 좌표: 장면을 정의하는 좌표
1. 뷰잉 좌표: 장면을 바라보는 가상 카메라 시점의 좌표 변환
1. 정규화 좌표: 일정 범위로 정규화된 좌표(-1 ~ 1)
1. 장치 좌표: 윈도우 내의 디스플레이 영역

### 2. 2차원 뷰잉 변환

세계 좌표에서 출력하고자 하는 범위(클리핑 윈도)를 정함

## 선분 클리핑

### 1. 클리핑이란

그림에서 원하는 영역을 벗어나는 부분을 잘라 내는 처리

### 2. 선분 클리핑

- 선분과 클리핑 윈도우가 교차하는 점들을 검출하여 클리핑 처리

### 3. Cohen-Sutherland 선분 클리핑

#### 끝점의 영역 코드

b3 :상 - b2 : 하 - b1 : 우 : b0 : 좌

클리핑 윈도우 안에 위치하면 영역 코드의 값이 0, 클리핑윈도우 밖이라면 1

이 영역코드를 이용하여 화면을 9개의 영역으로 구분할 수 있음

#### 양 끝점의 영역 코드에 따른 클리핑 영역 포함 여부의 판단

양 끝점을 bitwise or 연산을 통해 모든 값이 0인 경우 클리핑 윈도우 안에 있다고 판단

양 끝점을 bitwise and 연산을 통해 모든 값이 0이 아닌 경우 클리핑 윈도우 밖에 있다고 판단

선분과 교차되는 영역의 경계가 여러 개일 경우 클리핑을 반복해야 함

#### 선분과 클리핑 경계의 교점 계산

수직 경계와의 교점

- y = y_f + m(x_x_f)
- x = xw_min 또는 xw_max
- 직선의 방정식

수평 경계와의 교점

- x = x_f + (y-y_f)/m
- y = yw_min 또는 yw_max

### 4. Liang-Barsky 선분 클리핑

- Cohen-Sutherland처럼 여러번의 과정을 거치는 것이 아닌, 한번에 클리핑해주는 알고리즘
- 매개변수 선분 방정식을 이용
- a in > a out 인 경우 선분 전체가 클리핑 윈도우 외부에 있다는 것이므로 선분 전체를 제거

## 다각형 클리핑

### Shtherland-Hodgman 다각형 클리핑

- 좌, 우, 하, 상에 대한 클리핑을 차례로 수행
- 다각형을 구성하는 변 단위로 정해진 순서(CW, CCW)에 따라 처리기에 입력
- 두 꼭짓점이 클리핑 경계를 기준으로 위치하는 상황을 네 가지로 구분하여 처리함
- 각각의 경계에 대한 처리를 파이프라인 형식으로 처리할 수 있음
- 옥목 다각형을 클리핑할 경우 적절한 처리가 이루어지지 않을 거능성이 있음

