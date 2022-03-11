# 컴퓨터 그래픽스의 기본 요소

## DDA 알고리즘

- Digital Differential Analyzer
- |m|에 따라 기준 축을 정한 후, 기준 축의 좌표가 1만큼 변화할 때 나머지 축 좌표의 변화를 구하여 다음 점의 좌표를 계산
- |m| <= 1 인 경우: x축 좌표를 1씩 변화시킬 때, y축 좌표를 m만큼 변화시켜 다음 점의 좌표를 계산(기울기가 가파른 경우)
- |m| > 1인 경우: y축 좌표를 1씩 변화시킬 때, x축 좌표를 1/m만큼 변화시켜 다음 점의 좌표를 계산(기울기가 완만한 경우)
- 계산된 좌표를 반올림하여 구한 정수 좌표 위치에 점을 그림

### |Xend - X0| >= |Yend - y0| 
- 완만한 경우, x의 이동을 기준
- Xk+1 = Xl + 1, Yk+1 = Yk + m

### |Xend - x0| < |Yend - Y0| 
- 가파른 경우, y의 이동을 기준
- Yk+1 = Yl + 1, Xk+1 = Xk + 1/m

```cpp
/// <summary>
/// DDA 알고리즘
/// x0 != xEnd 이며 y0 != yEnd 라고 가정
/// </summary>
void DDA(int x0, int y0, int xEnd, int yEnd) {
    int dx = xEnd - x0, dy = yEnd - y0, steps, k;
    float xIncrement, yIncrement, x = x0, y = y0;

    if (abs(dx) > abs(dy)) steps = abs(dx);
    else                   steps = abs(dy);

    xIncrement = float(dx) / steps;
    yIncrement = float(dy) / steps;

    setPixel(round(x), round(y));
    for (k = 0; k < steps; k++) {
        x += xIncrement;
        y += yIncrement;
        setPixel(round(x), round(y));
    }
}
```

### 특성

- 기울기 값에 따라 한 축의 좌표는 1, 다른 축의 좌표는 m(또는 1/m)만큼 변화시키며 다음 좌표를 계산하여 가장 가까운 정수좌표에 해당하는 픽셀을 그림

### 문제점

- 부동소수점 계산을 해야 함(시간이 많이 소비됨)
- 긴 선분의 경우 부동소수점 연산의 오차가 누적되어 정확한 직선 경로를 벗어날 수 있음

## Bresenham의 직선 알고리즘

DDA알고리즘의 부동소수점 문제를 개선한 알고리즘

- 기울기가 0과 1 사이인 직선을 가정
- 기울기가 0과 1 사이에 해당하지 않는 경우, 0과 1사이에 있는것처럼 변환을 하여 연산
- 아래 정리에서는 기울기가 0과 1 사이인 경우만 생각
- 다음에 그릴 픽셀의 y위치를 계산된 y값이 중간값(0.5)에 해당하는지 판단하여 Yk에 그릴지, Yk+1에 그릴지 선택(중간점 선분 그리기 알고리즘)
- 직선의 방정식을 이용하여 계산

Xk+1 = Xk + 1<br>
Yk+1 = Yk or Yk+1 (기울기가 0과 1 사이이기 때문)

### 판별식

- 직선의 방정식을 이용해서 구함
- 중간점을 아래 판별식에 대입하여 다음의 y좌표 값을 구함
- 이전 단계에서 사용한 판별식을 이용하여 계산을 최소화함

F(x,y) = -2W(Y-Yl) + 2H(X - Xl)

F(x,y) < 0 -> (x,y)가 직선의 위에 있음<br>
F(x,y) > 0 -> (x,y)가 직선의 아래에 있음

```cpp
void bresenham_line(int xl, int yl, int xr, int yr) {
    // 0 < H/W < 1 이라고 가정
    int x, y = yl, W = xr - xl, H = yr - yl;
    int F = 2 * H - W, dF1 = 2 * H, dF2 = 2 * (H - W);

    for (x = xl; x <= xr; x++) {
        setPixel(x, y);
        if (F < 0) F += dF1;
        else { y++; F += dF2; }
    }
}
```