# Step9I — короткая карточка

```text
BUILD_STEP9I.bat -> ALL STEP9I BUILDS: SUCCESS
UPLOAD_STEP9I_FIELD_SERVICE.bat
OPEN_SERVICE_CONSOLE.bat
```

Порядок зелёных ворот:

```text
1 PRE-FLIGHT PASS
2 PROTOCOL 4/4
3 вручную P0.02=2, P0.03=9 на 4 ПЧ -> подтвердить в браузере
4 H1/H2/V1/V2 + и - -> Directions 8/8
5 pair H и pair Z -> Pairs 2/2
6 Assisted motion
```

Рабочие команды HE200:

```text
1000H = скорость 0..10000 = 0..100%
2000H=0001 FWD
2000H=0002 REV
2000H=0006 DECEL STOP
0005 FREE/COAST STOP НЕ ИСПОЛЬЗОВАТЬ
```

Лазеры:

```text
continuous / 5 Гц / 1 мм / 10 м
```

Потеря одного датчика пары:

```text
>650 мс  -> замедлять ОБА привода
>1200 мс -> дотормаживать до 0
>5000 мс -> FAULT
```

Красная кнопка браузера `ПЛАВНЫЙ STOP ВСЕХ` — сервисная команда, НЕ замена аппаратного E-STOP.


Маркер под воротами:
```text
зелёный -> текущий этап PASS, текст говорит следующий разрешённый шаг
красный NO-GO -> дальше не переходить, сохранить журнал
```

STOP:
```text
верхний красный -> все 4 привода
в Assisted motion -> только активная пара
```
