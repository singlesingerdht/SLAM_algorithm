import cv2
import numpy as np
import math
import rospy

void fitCircle(int N, float** points){
    float sum_x, sum_y = 0.0, 0.0;
    float sum_x2, sum_y2 = 0.0, 0.0;
    float sum_x3, sum_y3 = 0.0, 0.0;
    float sum_xy, sumx1y2, sumx2y1 = 0.0, 0.0, 0.0;
    float x2, y2, x3;
    for( int i=0; i<N; i++)
    {
        printf("%d", i);
        x = points[i][0];
        y = points[i][1];
        x2 = x * x;
        y2 = y * y;
        x3 = x * x * x;
        sum_x = sum_x + x;
        sum_y = sum_y + y
        sum_x2 = sum_x2 + x2
        sum_y2 = sum_y2 + y2
        sum_x3 = sum_x3 + x2 * x
        sum_y3 = sum_y3 + y2 * y
        sum_xy = sum_xy + x * y
        sumx1y2 = sumx1y2 + x * y2
        sumx2y1 = sumx2y1 + x2 * y
    }
    C = N * sum_x2 - sum_x * sum_x
    D = N * sum_xy - sum_x * sum_y
    E = N * sum_x3 + N * sumx1y2 - (sum_x2 + sum_y2) * sum_x
    G = N * sum_y2 - sum_y * sum_y
    H = N * sumx2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y
    a = (H * D - E * G) / (C * G - D * D)
    b = (H * C - E * D) / (D * D - G * C)
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N

    center_x = a / (-2)
    center_y = b / (-2)
    radius = math.sqrt(a * a + b * b - 4 * c) / 2
    return (int(center_x), int(center_y)), int(radius)
}

int main()
{
    img = cv2.imread("./circle.bmp")
    list = [[[168, 162], [179, 136], [204, 136], [219, 117], [242, 113], [255, 122], [289, 115], [319, 126], [332, 143], [357, 156]]]
    points = np.array(list, dtype=np.int64)

    // (x, y), (a, b), angle = cv2.fitEllipse(points)
    // print((x, y), (a, b), angle)
    // cv2.ellipse(img, ((x, y), (a, b), angle), (0, 255, 0), 2)

    (x, y), r = fitCircle(points)
    cv2.circle(img, (x, y), r, color=(255, 0, 0))
    cv2.imwrite("./fit_circle.bmp", img)
}