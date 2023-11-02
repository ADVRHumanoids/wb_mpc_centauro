# Benchmark computation time


### centauro_task.info (15 February 2023, commit `ab81f132a0f345edd30d28c21ddd36d2187e3079`)
constraints | &#10006; self-collision <br/> &#10006; joint position limits | :heavy_check_mark: self-collision <br/> &#10006; joint position limits | :heavy_check_mark: self-collision (both arms) <br/> &#10006; joint position limits | &#10006; self-collision <br/> :heavy_check_mark: joint position limits | :heavy_check_mark: self-collision <br/> :heavy_check_mark: joint position limits |
--- | --- | --- | --- |--- | ---|
Initialization [ms] |     1 | 2 | 2.1 | 6 | 8 |
LQ Approximation [ms] |   3 | 4 | 4.6 | 30| 35 |
Backward Pass [ms] |      21| 22| 23 | 79| 81 |
Compute Controller [ms] | 0 | 0 | 0.5 | 1 | 1.6|
Search Strategy [ms] |    3 | 4 | 4.5 | 12| 16.9|
Dual Solution [ms] |      0 | 0 | 0 | 0.23| 0.25|


### centauro_locomotion.info (16 February 2023, `static_walk` gait)
constraints | :heavy_check_mark: fixed steps <br/> &#10006; joint position limits <br/> &#10006; joint velocity limits | &#10006; fixed steps <br/> &#10006; joint position limits <br/> &#10006; joint velocity limits | :heavy_check_mark: fixed steps <br/> :heavy_check_mark: joint position limits <br/> &#10006; joint velocity limits | :heavy_check_mark: fixed steps <br/> :heavy_check_mark: joint position limits <br/> :heavy_check_mark: joint velocity limits |
--- | --- | --- | --- |--- |
Initialization [ms] |     2.5 | 2.4 | 3.2 | 4.5 |
LQ Approximation [ms] |   4.8 | 4.3 | 17.7| 33.8|
Backward Pass [ms] |      37  | 34  | 34.3| 35.5|
Compute Controller [ms] | 1.1 | 1.1 | 1   | 1   |
Search Strategy [ms] |    5.2 | 4.9 | 6.4 | 9.1 | 
Dual Solution [ms] |      0   |   0 | 0.1 | 0.2 |
