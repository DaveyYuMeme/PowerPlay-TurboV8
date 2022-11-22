# Notes

## On Quintic Splines

Quintic splines are defined by 6 equations:
```cpp
S0(t) = c0 + c1 * t + c2 * t ** 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5
S1(t) = c1 + 2 * c2 * t + 3 * c3 * t ** 2 + 4 * c4 * t ** 3 + 5 * c5 * t ** 4
S2(t) = 2 * c2 + 6 * c3 * t + 12 * c4 * t ** 2 + 20 * c5 * t ** 3
S3(t) = 6 * c3 + 24 * c4 * t + 60 * c5 * t ** 2
S4(t) = 24 * c4 + 120 * c5 * t
S5(t) = 120 * c5
```

C^N consistency means that two end-to-end quintic splines have the same
value for the Nth equation.