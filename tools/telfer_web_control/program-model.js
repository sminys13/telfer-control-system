(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.ProgramModel = api;
})(globalThis, function () {
  'use strict';
  const MAX_ZONES = 10;
  const copy = value => JSON.parse(JSON.stringify(value));
  const int = (value, min, max, label) => {
    if (value === '' || value === null || value === undefined || !/^-?\d+$/.test(String(value))) throw Error(`${label}: требуется целое число`);
    const result = Number(value);
    if (!Number.isSafeInteger(result) || result < min || result > max) throw Error(`${label}: ${min}…${max}`);
    return result;
  };
  function normalize(source) {
    if (!source || !Array.isArray(source.zones) || !source.zones.length || source.zones.length > MAX_ZONES) throw Error('В программе должно быть от 1 до 10 зон');
    const result = {
      slot: int(source.slot ?? 1, 1, 4, 'Программа'), name: String(source.name ?? 'Программа ' + (source.slot ?? 1)),
      home1: int(source.home1, 0, 10000, 'HOME X1'), home2: int(source.home2, 0, 10000, 'HOME X2'),
      travel1: int(source.travel1, 0, 10000, 'Транспорт Z1'), travel2: int(source.travel2, 0, 10000, 'Транспорт Z2'),
      drip: int(source.drip ?? 2, 0, 36000, 'Стекание'), tiltPct: int(source.tiltPct ?? 10, 5, 80, 'Скорость наклона'), lowSide: int(source.lowSide ?? 0, 0, 1, 'Низкая сторона'),
      zones: source.zones.map((z, i) => ({
        n: i + 1, enabled: int(z.enabled ?? z.en ?? 1, 0, 1, `Зона ${i + 1}: включение`),
        x1: int(z.x1, 0, 10000, `Зона ${i + 1}: X1`), x2: int(z.x2, 0, 10000, `Зона ${i + 1}: X2`),
        z1: int(z.z1, 0, 10000, `Зона ${i + 1}: Z1`), z2: int(z.z2, 0, 10000, `Зона ${i + 1}: Z2`),
        dip: int(z.dip ?? 2, 0, 36000, `Зона ${i + 1}: погружение`), tilt: int(z.tilt ?? 0, 0, 5000, `Зона ${i + 1}: наклон`),
        wait: int(z.wait ?? 0, 0, 36000, `Зона ${i + 1}: пауза`), hPct: int(z.hPct ?? z.hpct ?? 10, 5, 80, `Зона ${i + 1}: скорость X`), vPct: int(z.vPct ?? z.vpct ?? 10, 5, 80, `Зона ${i + 1}: скорость Z`)
      }))
    };
    if (new TextEncoder().encode(result.name).length > 39) throw Error('Слишком длинное название: максимум 39 байт UTF-8 (около 19 русских букв)');
    if (/[\u0000-\u001f\u007f]/.test(result.name)) throw Error('В названии есть служебные символы');
    result.order = (source.order ?? result.zones.map((_, i) => i + 1)).map(n => int(n, 1, result.zones.length, 'Порядок'));
    if (result.order.length !== result.zones.length || new Set(result.order).size !== result.zones.length) throw Error('Каждая зона должна встречаться в порядке ровно один раз');
    const dry = source.drying ?? {};
    result.drying = {enabled: int(dry.enabled ?? 0, 0, 1, 'Сушка'), seconds: int(dry.seconds ?? 60, 0, 36000, 'Время сушки'), staging: int(dry.staging ?? 1, 1, result.zones.length, 'Промежуточная зона'),
      x1: int(dry.x1 ?? result.home1, 0, 10000, 'Сушка X1'), x2: int(dry.x2 ?? result.home2, 0, 10000, 'Сушка X2'), z1: int(dry.z1 ?? result.travel1, 0, 10000, 'Сушка Z1'), z2: int(dry.z2 ?? result.travel2, 0, 10000, 'Сушка Z2')};
    return result;
  }
  function ready(program) {
    const p = normalize(program);
    if (!p.zones.some(z => z.enabled)) throw Error('Включите хотя бы одну зону');
    return p;
  }
  function demo(slot = 1, positions = [1000, 1000, 1000, 1000], dx = 50, dz = 30, wait = 1) {
    const [x1, x2, z1, z2] = positions;
    return normalize({slot, name: `Учебная ${slot}`, home1: x1, home2: x2, travel1: z1, travel2: z2, drip: wait, tiltPct: 10, lowSide: 0,
      zones: [0, 1].map((n) => ({enabled: 1, x1: x1 + n * dx, x2: x2 + n * dx, z1: z1 - dz, z2: z2 - dz, dip: wait, tilt: 0, wait: 0, hPct: 10, vPct: 10}))});
  }
  function move(program, zone, direction) {
    const p = normalize(program), i = p.order.indexOf(zone), target = i + direction;
    if (i < 0 || target < 0 || target >= p.order.length) return p;
    [p.order[i], p.order[target]] = [p.order[target], p.order[i]];
    return p;
  }
  function duplicate(program, zone) {
    const p = normalize(program);
    if (p.zones.length === MAX_ZONES) throw Error('Максимум 10 зон');
    const z = p.zones[zone - 1];if (!z) throw Error('Выберите зону');
    const n = p.zones.length + 1;
    p.zones.push({...z, n});p.order.splice(p.order.indexOf(zone) + 1, 0, n);
    return p;
  }
  function remove(program, zone) {
    const p = normalize(program);
    if (p.zones.length === 1) throw Error('В программе должна остаться хотя бы одна зона');
    if (!p.zones[zone - 1]) throw Error('Выберите зону');
    p.zones.splice(zone - 1, 1);p.zones.forEach((z, i) => z.n = i + 1);
    p.order = p.order.filter(n => n !== zone).map(n => n > zone ? n - 1 : n);
    p.drying.staging = Math.max(1, Math.min(p.zones.length, p.drying.staging > zone ? p.drying.staging - 1 : p.drying.staging));
    return p;
  }
  function resize(program, count) {
    let p = normalize(program);count = int(count, 1, MAX_ZONES, 'Количество зон');
    while (p.zones.length < count) {
      const n = p.zones.length + 1, previous = p.zones.at(-1);
      p.zones.push({...previous, n, x1: Math.min(10000, previous.x1 + 50), x2: Math.min(10000, previous.x2 + 50)});p.order.push(n);
    }
    while (p.zones.length > count) p = remove(p, p.zones.length);
    return p;
  }
  function timed(program) {
    const p = normalize(program);
    return p.zones.filter(z => z.enabled).reduce((sum, z) => sum + z.dip + z.wait + p.drip, 0) + (p.drying.enabled ? p.drying.seconds : 0);
  }
  return {MAX_ZONES, copy, normalize, ready, demo, move, duplicate, remove, resize, timed};
});
