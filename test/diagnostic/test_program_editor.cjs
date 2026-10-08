const assert = require('node:assert/strict');
const model = require('../../tools/telfer_web_control/program-model.js');
const base = model.demo();
assert.equal(model.ready(base).zones.length, 2);
assert.equal(model.timed(base), 4);
const moved = model.move(base, 2, -1);assert.deepEqual(moved.order, [2, 1]);assert.equal(moved.zones[1].x1, 1050);
const copied = model.duplicate(moved, 2);assert.deepEqual(copied.order, [2, 3, 1]);assert.equal(copied.zones[2].x1, 1050);
copied.drying.staging = 3;const removed = model.remove(copied, 1);assert.deepEqual(removed.order, [1, 2]);assert.equal(removed.drying.staging, 2);
const ten = model.resize(base, 10);assert.equal(ten.zones.length, 10);assert.equal(ten.order.at(-1), 10);
assert.throws(() => model.resize(base, 11));assert.equal(model.resize(ten, 1).zones.length, 1);
assert.throws(() => model.normalize({...base, order: [1, 1]}));assert.throws(() => model.normalize({...base, home1: ''}));
assert.throws(() => model.normalize({...base, name: 'а'.repeat(20)}));
assert.throws(() => model.normalize({...base, name: '<x>\n'}));
assert.throws(() => model.ready({...base, zones: base.zones.map(z => ({...z, enabled: 0}))}));
assert.equal(model.normalize({...base, zones: base.zones.map(z => ({...z, dip: 36000}))}).zones[0].dip, 36000);
assert.deepEqual(model.normalize(JSON.parse(JSON.stringify(base))), base);
assert.deepEqual(base.order, [1, 2]); // operations do not mutate their input
console.log('PASS: zone reorder/duplicate/delete/resize 1..10, stage remap, ranges/UTF-8 name, all-disabled gate, 36000 s, JSON roundtrip');
