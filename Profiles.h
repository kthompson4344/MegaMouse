const int curve1Time = 385;

float curve1[curve1Time] = {
0,
0.01,
0.03,
0.06,
0.1,
0.15,
0.21,
0.28,
0.36,
0.45,
0.55,
0.66,
0.78,
0.91,
1.05,
1.2,
1.36,
1.53,
1.71,
1.9,
2.1,
2.31,
2.53,
2.76,
3,
3.25,
3.51,
3.78,
4.06,
4.35,
4.65,
4.96,
5.28,
5.61,
5.95,
6.3,
6.66,
7.03,
7.41,
7.8,
8.2,
8.61,
9.03,
9.46,
9.9,
10.35,
10.81,
11.28,
11.76,
12.25,
12.75,
13.25,
13.75,
14.25,
14.75,
15.25,
15.75,
16.25,
16.75,
17.25,
17.75,
18.25,
18.75,
19.25,
19.75,
20.25,
20.75,
21.25,
21.75,
22.25,
22.75,
23.25,
23.75,
24.25,
24.75,
25.25,
25.75,
26.25,
26.75,
27.25,
27.75,
28.25,
28.75,
29.25,
29.75,
30.25,
30.75,
31.25,
31.75,
32.25,
32.75,
33.25,
33.75,
34.25,
34.75,
35.25,
35.75,
36.25,
36.75,
37.25,
37.75,
38.25,
38.75,
39.25,
39.75,
40.25,
40.75,
41.25,
41.75,
42.25,
42.75,
43.25,
43.75,
44.25,
44.75,
45.25,
45.75,
46.25,
46.75,
47.25,
47.75,
48.25,
48.75,
49.25,
49.75,
50.25,
50.75,
51.25,
51.75,
52.25,
52.75,
53.25,
53.75,
54.25,
54.75,
55.25,
55.75,
56.25,
56.75,
57.25,
57.75,
58.25,
58.75,
59.25,
59.75,
60.25,
60.75,
61.25,
61.75,
62.25,
62.75,
63.25,
63.75,
64.25,
64.75,
65.25,
65.75,
66.25,
66.75,
67.25,
67.75,
68.25,
68.75,
69.25,
69.75,
70.25,
70.75,
71.25,
71.75,
72.25,
72.75,
73.25,
73.75,
74.25,
74.75,
75.25,
75.75,
76.25,
76.75,
77.25,
77.75,
78.24,
78.72,
79.19,
79.65,
80.1,
80.54,
80.97,
81.39,
81.8,
82.2,
82.59,
82.97,
83.34,
83.7,
84.05,
84.39,
84.72,
85.04,
85.35,
85.65,
85.94,
86.22,
86.49,
86.75,
87,
87.24,
87.47,
87.69,
87.9,
88.1,
88.29,
88.47,
88.64,
88.8,
88.95,
89.09,
89.22,
89.34,
89.45,
89.55,
89.64,
89.72,
89.79,
89.85,
89.9,
89.94,
89.97,
89.99,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90,
90};

const int curve2Time = 201;
float curve2[curve2Time] = {
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0, 
0.015, 
0.045, 
0.09, 
0.15, 
0.225, 
0.315, 
0.42, 
0.54, 
0.675, 
0.825, 
0.99, 
1.17, 
1.365, 
1.575, 
1.8, 
2.04, 
2.295, 
2.565, 
2.85, 
3.15, 
3.465, 
3.795, 
4.14, 
4.5, 
4.875, 
5.265, 
5.67, 
6.09, 
6.525, 
6.975, 
7.44, 
7.92, 
8.415, 
8.925, 
9.45, 
9.99, 
10.545, 
11.115, 
11.7, 
12.3, 
12.915, 
13.545, 
14.19, 
14.85, 
15.525, 
16.215, 
16.92, 
17.64, 
18.375, 
19.125, 
19.875, 
20.625, 
21.375, 
22.125, 
22.875, 
23.625, 
24.375, 
25.125, 
25.875, 
26.625, 
27.375, 
28.125, 
28.875, 
29.625, 
30.375, 
31.125, 
31.875, 
32.625, 
33.375, 
34.125, 
34.875, 
35.625, 
36.375, 
37.125, 
37.875, 
38.625, 
39.375, 
40.125, 
40.875, 
41.625, 
42.375, 
43.125, 
43.875, 
44.625, 
45.375, 
46.125, 
46.875, 
47.625, 
48.375, 
49.125, 
49.875, 
50.625, 
51.375, 
52.125, 
52.875, 
53.625, 
54.375, 
55.125, 
55.875, 
56.625, 
57.375, 
58.125, 
58.875, 
59.625, 
60.375, 
61.125, 
61.875, 
62.625, 
63.375, 
64.125, 
64.875, 
65.625, 
66.375, 
67.125, 
67.875, 
68.625, 
69.375, 
70.125, 
70.875, 
71.625, 
72.36, 
73.08, 
73.785, 
74.475, 
75.15, 
75.81, 
76.455, 
77.085, 
77.7, 
78.3, 
78.885, 
79.455, 
80.01, 
80.55, 
81.075, 
81.585, 
82.08, 
82.56, 
83.025, 
83.475, 
83.91, 
84.33, 
84.735, 
85.125, 
85.5, 
85.86, 
86.205, 
86.535, 
86.85, 
87.15, 
87.435, 
87.705, 
87.96, 
88.2, 
88.425, 
88.635, 
88.83, 
89.01, 
89.175, 
89.325, 
89.46, 
89.58, 
89.685, 
89.775, 
89.85, 
89.91, 
89.955, 
89.985, 
90, 
90};

const int curve3Time = 277;
int curve3[curve3Time] = {
0, 
10, 
20, 
30, 
40, 
50, 
60, 
70, 
80, 
90, 
100, 
110, 
120, 
130, 
140, 
150, 
160, 
170, 
180, 
190, 
200, 
210, 
220, 
230, 
240, 
250, 
260, 
270, 
280, 
290, 
300, 
310, 
320, 
330, 
340, 
350, 
360, 
370, 
380, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
378, 
368, 
358, 
348, 
338, 
328, 
318, 
308, 
298, 
288, 
278, 
268, 
258, 
248, 
238, 
228, 
218, 
208, 
198, 
188, 
178, 
168, 
158, 
148, 
138, 
128, 
118, 
108, 
98, 
88, 
78, 
68, 
58, 
48, 
38, 
28, 
18, 
8, 
0};
