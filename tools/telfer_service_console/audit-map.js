"use strict";
const HE200_AUDIT_MAP = [
  {
    "name": "P0.01",
    "reg": 61441,
    "label": "Режим двигателя",
    "expected": null
  },
  {
    "name": "P0.02",
    "reg": 61442,
    "label": "Источник команды",
    "expected": 2
  },
  {
    "name": "P0.03",
    "reg": 61443,
    "label": "Источник частоты A",
    "expected": 9
  },
  {
    "name": "P0.04",
    "reg": 61444,
    "label": "Источник частоты B",
    "expected": null
  },
  {
    "name": "P0.07",
    "reg": 61447,
    "label": "Комбинация источников частоты",
    "expected": null
  },
  {
    "name": "P0.09",
    "reg": 61449,
    "label": "Инверсия направления",
    "expected": null
  },
  {
    "name": "P0.10",
    "reg": 61450,
    "label": "Максимальная частота",
    "expected": null
  },
  {
    "name": "P0.11",
    "reg": 61451,
    "label": "Источник верхнего предела",
    "expected": null
  },
  {
    "name": "P0.12",
    "reg": 61452,
    "label": "Верхний предел частоты",
    "expected": null
  },
  {
    "name": "P0.14",
    "reg": 61454,
    "label": "Нижний предел частоты",
    "expected": null
  },
  {
    "name": "P0.17",
    "reg": 61457,
    "label": "Время разгона",
    "expected": null
  },
  {
    "name": "P0.18",
    "reg": 61458,
    "label": "Время торможения",
    "expected": null
  },
  {
    "name": "P0.19",
    "reg": 61459,
    "label": "Единица времени разгона",
    "expected": null
  },
  {
    "name": "P0.22",
    "reg": 61462,
    "label": "Разрешение частоты",
    "expected": null
  },
  {
    "name": "P0.24",
    "reg": 61464,
    "label": "Выбор двигателя",
    "expected": null
  },
  {
    "name": "P0.25",
    "reg": 61465,
    "label": "База разгона",
    "expected": null
  },
  {
    "name": "P0.27",
    "reg": 61467,
    "label": "Привязка источников",
    "expected": 0
  },
  {
    "name": "P0.28",
    "reg": 61468,
    "label": "Режим связи",
    "expected": null
  },
  {
    "name": "P3.13",
    "reg": 62221,
    "label": "Источник напряжения V/F",
    "expected": null
  },
  {
    "name": "P3.14",
    "reg": 62222,
    "label": "Уставка напряжения V/F",
    "expected": null
  },
  {
    "name": "P4.00",
    "reg": 62464,
    "label": "Назначение X1",
    "expected": 1
  },
  {
    "name": "P4.01",
    "reg": 62465,
    "label": "Назначение X2",
    "expected": 2
  },
  {
    "name": "P4.02",
    "reg": 62466,
    "label": "Назначение X3",
    "expected": 10
  },
  {
    "name": "P4.03",
    "reg": 62467,
    "label": "Назначение X4",
    "expected": 9
  },
  {
    "name": "P4.04",
    "reg": 62468,
    "label": "Назначение X5 (зависит от ревизии)",
    "expected": null
  },
  {
    "name": "P4.05",
    "reg": 62469,
    "label": "Назначение X6 (зависит от ревизии)",
    "expected": null
  },
  {
    "name": "P4.10",
    "reg": 62474,
    "label": "Фильтр входов",
    "expected": null
  },
  {
    "name": "P4.11",
    "reg": 62475,
    "label": "Режим клемм",
    "expected": 0
  },
  {
    "name": "P4.35",
    "reg": 62499,
    "label": "Задержка X1",
    "expected": null
  },
  {
    "name": "P4.36",
    "reg": 62500,
    "label": "Задержка X2",
    "expected": null
  },
  {
    "name": "P4.37",
    "reg": 62501,
    "label": "Задержка X3",
    "expected": null
  },
  {
    "name": "P4.38",
    "reg": 62502,
    "label": "Полярность X1–X4",
    "expected": null
  },
  {
    "name": "P4.39",
    "reg": 62503,
    "label": "Полярность доп. входов (ревизия)",
    "expected": null
  },
  {
    "name": "P5.02",
    "reg": 62722,
    "label": "Назначение реле",
    "expected": null
  },
  {
    "name": "P5.18",
    "reg": 62738,
    "label": "Задержка реле",
    "expected": null
  },
  {
    "name": "P5.22",
    "reg": 62742,
    "label": "Полярность выходов",
    "expected": null
  },
  {
    "name": "P6.00",
    "reg": 62976,
    "label": "Режим пуска",
    "expected": null
  },
  {
    "name": "P6.01",
    "reg": 62977,
    "label": "Поиск скорости",
    "expected": null
  },
  {
    "name": "P6.03",
    "reg": 62979,
    "label": "Стартовая частота",
    "expected": null
  },
  {
    "name": "P6.04",
    "reg": 62980,
    "label": "Выдержка стартовой частоты",
    "expected": null
  },
  {
    "name": "P6.05",
    "reg": 62981,
    "label": "Ток DC / предвозбуждения",
    "expected": null
  },
  {
    "name": "P6.06",
    "reg": 62982,
    "label": "Время DC / предвозбуждения",
    "expected": null
  },
  {
    "name": "P6.07",
    "reg": 62983,
    "label": "Кривая разгона",
    "expected": null
  },
  {
    "name": "P6.10",
    "reg": 62986,
    "label": "Способ остановки",
    "expected": null
  },
  {
    "name": "P6.11",
    "reg": 62987,
    "label": "Начало DC торможения",
    "expected": null
  },
  {
    "name": "P6.12",
    "reg": 62988,
    "label": "Задержка DC торможения",
    "expected": null
  },
  {
    "name": "P6.13",
    "reg": 62989,
    "label": "Ток DC торможения",
    "expected": null
  },
  {
    "name": "P6.14",
    "reg": 62990,
    "label": "Время DC торможения",
    "expected": null
  },
  {
    "name": "P7.10",
    "reg": 63242,
    "label": "Версия ПО 1",
    "expected": null
  },
  {
    "name": "P7.11",
    "reg": 63243,
    "label": "Версия ПО 2",
    "expected": null
  },
  {
    "name": "P8.12",
    "reg": 63500,
    "label": "Пауза смены направления",
    "expected": null
  },
  {
    "name": "P8.13",
    "reg": 63501,
    "label": "Запрет реверса",
    "expected": null
  },
  {
    "name": "P8.14",
    "reg": 63502,
    "label": "Режим ниже нижнего предела",
    "expected": null
  },
  {
    "name": "P8.18",
    "reg": 63506,
    "label": "Защита пуска",
    "expected": null
  },
  {
    "name": "A0.00",
    "reg": 40960,
    "label": "Скорость / момент",
    "expected": null
  },
  {
    "name": "A0.01",
    "reg": 40961,
    "label": "Источник момента",
    "expected": null
  },
  {
    "name": "A0.03",
    "reg": 40963,
    "label": "Задание момента",
    "expected": null
  },
  {
    "name": "A0.05",
    "reg": 40965,
    "label": "Предел FWD в режиме момента",
    "expected": null
  },
  {
    "name": "A0.06",
    "reg": 40966,
    "label": "Предел REV в режиме момента",
    "expected": null
  },
  {
    "name": "A1.00",
    "reg": 41216,
    "label": "Виртуальный вход VX1",
    "expected": null
  },
  {
    "name": "A1.01",
    "reg": 41217,
    "label": "Виртуальный вход VX2",
    "expected": null
  },
  {
    "name": "A1.02",
    "reg": 41218,
    "label": "Виртуальный вход VX3",
    "expected": null
  },
  {
    "name": "A1.03",
    "reg": 41219,
    "label": "Виртуальный вход VX4",
    "expected": null
  },
  {
    "name": "A1.04",
    "reg": 41220,
    "label": "Виртуальный вход VX5",
    "expected": null
  },
  {
    "name": "A1.05",
    "reg": 41221,
    "label": "Режим виртуальных входов",
    "expected": null
  },
  {
    "name": "A1.06",
    "reg": 41222,
    "label": "Состояние виртуальных входов",
    "expected": null
  },
  {
    "name": "Pd.00",
    "reg": 64768,
    "label": "Скорость связи",
    "expected": null
  },
  {
    "name": "Pd.01",
    "reg": 64769,
    "label": "Формат связи",
    "expected": null
  },
  {
    "name": "Pd.02",
    "reg": 64770,
    "label": "Адрес ПЧ",
    "expected": null
  },
  {
    "name": "Pd.03",
    "reg": 64771,
    "label": "Задержка ответа",
    "expected": null
  },
  {
    "name": "Pd.04",
    "reg": 64772,
    "label": "Таймаут связи",
    "expected": null
  },
  {
    "name": "Pd.05",
    "reg": 64773,
    "label": "Формат Modbus",
    "expected": null
  },
  {
    "name": "STATUS",
    "reg": 12288,
    "label": "RUN/STOP (1/2/3)",
    "expected": null
  },
  {
    "name": "COMM",
    "reg": 4096,
    "label": "Задание связи (знаковое)",
    "expected": null
  },
  {
    "name": "DI_ALT",
    "reg": 4104,
    "label": "Входы по Appendix A (raw)",
    "expected": null
  },
  {
    "name": "RUN_HZ",
    "reg": 28672,
    "label": "Выходная частота 0.01 Гц",
    "expected": null
  },
  {
    "name": "SET_HZ",
    "reg": 28673,
    "label": "Заданная частота 0.01 Гц",
    "expected": null
  },
  {
    "name": "BUS_V",
    "reg": 28674,
    "label": "Шина DC 0.1 В",
    "expected": null
  },
  {
    "name": "OUT_V",
    "reg": 28675,
    "label": "Выходное напряжение В",
    "expected": null
  },
  {
    "name": "OUT_I",
    "reg": 28676,
    "label": "Выходной ток 0.01 А",
    "expected": null
  },
  {
    "name": "DI",
    "reg": 28679,
    "label": "Входы D0.07 (raw)",
    "expected": null
  },
  {
    "name": "DO",
    "reg": 28680,
    "label": "Выходы (raw)",
    "expected": null
  },
  {
    "name": "COMM_D",
    "reg": 28700,
    "label": "Задание связи D0.28",
    "expected": null
  },
  {
    "name": "FREQ_A",
    "reg": 28702,
    "label": "Частота A",
    "expected": null
  },
  {
    "name": "FREQ_B",
    "reg": 28703,
    "label": "Частота B",
    "expected": null
  },
  {
    "name": "DI_VIEW",
    "reg": 28713,
    "label": "Индикация входов (raw)",
    "expected": null
  },
  {
    "name": "DO_VIEW",
    "reg": 28714,
    "label": "Индикация выходов (raw)",
    "expected": null
  },
  {
    "name": "FUNC1",
    "reg": 28715,
    "label": "Функции 01–40 (raw)",
    "expected": null
  },
  {
    "name": "FUNC2",
    "reg": 28716,
    "label": "Функции 41–80 (raw)",
    "expected": null
  },
  {
    "name": "FAULT",
    "reg": 28717,
    "label": "Текущая ошибка",
    "expected": null
  },
  {
    "name": "SET_PCT",
    "reg": 28731,
    "label": "Уставка %",
    "expected": null
  },
  {
    "name": "RUN_PCT",
    "reg": 28732,
    "label": "Выходная частота % (знаковое)",
    "expected": null
  },
  {
    "name": "STATE",
    "reg": 28733,
    "label": "Состояние D0.61 (raw)",
    "expected": null
  }
];
