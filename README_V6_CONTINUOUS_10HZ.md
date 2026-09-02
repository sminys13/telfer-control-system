# V6 continuous 10 Hz experiment

Эта версия проверяет более стабильный режим непрерывного измерения:

```text
LASER ON
SET RANGE 10m
SET RESOLUTION 1mm
SET FREQUENCY 10Hz
CONTINUOUS
```

Команда частоты 10 Гц:

```text
04 0A 0A EE
```

Почему не 20 Гц: по полевому тесту 20 Гц дал больше миганий и залипаний. 10 Гц должен быть компромиссом между скоростью и устойчивостью.

Ошибки на DWIN:

```text
1  = X1 invalid
2  = X2 invalid
4  = Z1 invalid
8  = Z2 invalid
12 = Z1+Z2 invalid
13 = X1+Z1+Z2 invalid
14 = X2+Z1+Z2 invalid
15 = все 4 invalid
```
