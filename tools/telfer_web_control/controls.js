'use strict';
function stopJogTimer() {if (jogTimer) clearInterval(jogTimer); jogTimer = null;}
async function beginJog(button, e) {
  e.preventDefault(); if (jog || !armed() || busy) return;
  try {const pct = number('jogPct', 5, 80); jog = {target: button.dataset.jog, dir: button.dataset.dir, pct, button}; button.classList.add('held'); button.setPointerCapture?.(e.pointerId); await request(`web jog ${token()} ${jog.target} ${jog.dir} ${pct}`); if (!jog) return; jogTimer = setInterval(() => {if (jog) write(`web jog ${controller.session} ${jog.target} ${jog.dir} ${jog.pct}`).catch(() => releaseJog());}, 180);} catch (error) {note(error.message, true); releaseJog();} render();
}
async function releaseJog() {if (!jog) return; jog.button?.classList.remove('held'); jog = null; stopJogTimer(); try {await write(`web release ${controller.session}`, true);} catch (e) {note(e.message, true);} render();}
function scale(def, drive) {
  if (['P0.17', 'P0.18'].includes(def[0])) {const v = parameters.get(`${drive}:P0.19`); return v?.ok === '1' ? ({0: 1, 1: 10, 2: 100})[Number(v.raw)] : null;}
  if (['P0.10', 'P0.14', 'P6.03'].includes(def[0])) {const v = parameters.get(`${drive}:P0.22`); return v?.ok === '1' ? ({1: 10, 2: 100})[Number(v.raw)] : null;}
  return def[2];
}
function parameterGroup(key) {return key.startsWith('P6.') ? 'startup' : key.startsWith('P4.') || key.startsWith('P8.') ? 'inputs' : 'main';}
function filterParameters() {const group = $('paramGroup').value; $('params').querySelectorAll('tr').forEach(row => row.hidden = group !== 'all' && row.dataset.group !== group);}
function renderParameter(v) {
  const def = paramDefs.find(d => d[0] === v.key), cell = document.querySelector(`[data-param="${v.key}"][data-drive="${v.drive}"]`); if (!cell || !def) return;
  const divisor = scale(def, v.drive), input = cell.querySelector('input'); input.value = v.ok === '1' && divisor ? Number(v.raw) / divisor : ''; input.step = divisor ? 1 / divisor : 1;
  cell.querySelector('.raw').textContent = v.ok === '1' ? `raw ${v.raw}` : `Ошибка ${v.error}; ex ${v.exception}`;
  input.disabled = def[5] === false || !divisor; const button = cell.querySelector('button'); if (button) button.disabled = !armed() || busy || moving() || !divisor;
}
async function readParameters() {
  const group = $('paramGroup').value, required = new Set(['P0.19', 'P0.22', ...paramDefs.filter(d => group === 'all' || parameterGroup(d[0]) === group).map(d => d[0])]);
  const defs = [...paramDefs].filter(d => required.has(d[0])).sort((a, b) => ['P0.19', 'P0.22'].includes(a[0]) ? -1 : ['P0.19', 'P0.22'].includes(b[0]) ? 1 : 0);
  let count = 0, failed = 0;
  for (const def of defs) for (const drive of names) {if (!connected()) throw Error('Подключение потеряно'); $('paramProgress').textContent = `Чтение ${++count} / ${defs.length * 4}`; try {await request(`web read ${drive} ${def[0]}`, '@WEB_PARAM ');} catch (e) {if (e.message.startsWith('Команда отменена')) throw e; failed++; note(`${drive.toUpperCase()} ${def[0]}: ${e.message}`, true);}}
  $('paramProgress').textContent = `Прочитано ${count - failed} / ${count}${isSim() ? ' · параметры модели' : ''}`; for (const value of parameters.values()) renderParameter(value);
}
async function writeParameter(button) {
  const cell = button.closest('[data-param]'), key = cell.dataset.param, drive = cell.dataset.drive, def = paramDefs.find(d => d[0] === key), before = parameters.get(`${drive}:${key}`), divisor = scale(def, drive);
  if (before?.ok !== '1' || !divisor) throw Error('Сначала прочитайте параметр и его шкалу');
  const text = cell.querySelector('input').value.trim(), value = Number(text);
  if (!/^\d+(\.\d+)?$/.test(text) || !Number.isFinite(value) || value < def[3] || value > def[4] || Math.abs(value * divisor - Math.round(value * divisor)) > 1e-7) throw Error('Значение вне диапазона или шага');
  await request(`web write ${token()} ${drive} ${key} ${before.raw} ${Math.round(value * divisor)}`); await request(`web read ${drive} ${key}`, '@WEB_PARAM ');
}
function updateSettingsInputs(i, v) {
  const card = document.querySelector(`[data-profile-card="${names[i]}"]`), sensor = document.querySelector(`[data-sensor-card="${names[i]}"]`);
  if (card && card.dataset.dirty !== '1' && !card.contains(document.activeElement)) for (const input of card.querySelectorAll('input')) if (v[input.dataset.key] !== undefined) input.value = v[input.dataset.key];
  if (sensor && sensor.dataset.dirty !== '1' && !sensor.contains(document.activeElement)) {sensor.querySelector('[data-offset]').value = v.offset; sensor.querySelector('[data-inverse]').checked = v.invert === '1';}
}
function download(name, text, type = 'text/plain') {const url = URL.createObjectURL(new Blob([text], {type})), a = document.createElement('a'); a.href = url; a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);}
function buildControls() {
  names.forEach((d, i) => {
    $('axes').insertAdjacentHTML('beforeend', `<div class="axis"><b>${labels[i]} <small>${d.toUpperCase()}</small></b><div class="value">—</div><div class="state">Нет данных</div><div class="hz">Выход —</div><div class="fault"></div></div>`);
    $('calCards').insertAdjacentHTML('beforeend', `<div class="card"><h3>${d.toUpperCase()} · ${labels[i]}</h3><div class="toolbar"><button data-calibrate="${d}" data-dir="fwd">FWD</button><button data-calibrate="${d}" data-dir="rev">REV</button></div><p class="result">Нужны измерения</p></div>`);
    $('profiles').insertAdjacentHTML('beforeend', `<fieldset data-profile-card="${d}"><legend>${d.toUpperCase()}</legend>${[['manual', 'Ручная, %', 10], ['max', 'Максимальная, %', 40], ['slow', 'Медленная, %', 10], ['band', 'Замедление, мм', 100], ['tolerance', 'Допуск, мм', 8]].map(([key, label, value]) => `<label>${label}<input data-key="${key}" type="number" value="${value}"></label>`).join('')}<button data-profile="${d}">Применить</button></fieldset>`);
    $('sensorSettings').insertAdjacentHTML('beforeend', `<fieldset data-sensor-card="${d}"><legend>${labels[i]}</legend><label>Смещение, мм<input data-offset type="number" value="0"></label><label><input data-inverse type="checkbox"> Инверсия</label><button data-sensor="${d}">Применить</button></fieldset>`);
  });
  for (const [target, mask, label] of [['h', 3, 'Горизонтальная пара'], ['v', 12, 'Вертикальная пара'], ['h1', 1, 'H1'], ['h2', 2, 'H2'], ['v1', 4, 'V1'], ['v2', 8, 'V2']]) $('jogCards').insertAdjacentHTML('beforeend', `<div class="card"><h3>${label}</h3><button class="jog" data-jog="${target}" data-mask="${mask}" data-dir="neg">−</button> <button class="jog" data-jog="${target}" data-mask="${mask}" data-dir="pos">+</button></div>`);
  for (const def of paramDefs) {
    const row = document.createElement('tr'); row.dataset.group = parameterGroup(def[0]);
    row.innerHTML = `<th>${def[0]}</th><td>${def[1]}</td>` + names.map(d => `<td data-param="${def[0]}" data-drive="${d}"><div class="paramCell"><input aria-label="${d} ${def[0]}" type="number" step="${1 / def[2]}" min="${def[3]}" max="${def[4]}" ${def[5] === false ? 'disabled' : ''}>${def[5] === false ? '' : '<button data-write-param title="Записать и проверить">✓</button>'}</div><span class="raw">Не прочитано</span></td>`).join(''); $('params').appendChild(row);
  }
}
function bindControls() {
  document.querySelectorAll('[data-tab]').forEach(b => b.addEventListener('click', () => selectTab(b.dataset.tab)));
  $('connect').addEventListener('click', connect); $('disconnect').addEventListener('click', disconnect); $('stop').addEventListener('click', stop); $('external').addEventListener('change', render);
  $('arm').addEventListener('click', () => action(async () => {if (!isSim() && !$('external').checked) throw Error('Подтвердите аппаратную защиту'); localStop = false; await request(`web arm ${controller.session}`); event(isSim() ? 'Работа с моделью разрешена' : 'Управление разрешено');}));
  $('clearSafety').addEventListener('click', () => action(() => request(`web clear ${controller.session}`)));
  document.querySelectorAll('[data-jog]').forEach(b => {b.addEventListener('pointerdown', e => beginJog(b, e)); for (const eventName of ['pointerup', 'pointercancel', 'lostpointercapture']) b.addEventListener(eventName, releaseJog); b.addEventListener('contextmenu', e => e.preventDefault());});
  document.querySelectorAll('[data-calibrate]').forEach(b => b.addEventListener('click', () => action(() => request(`web calibrate ${token()} ${b.dataset.calibrate} ${b.dataset.dir} ${number('calPct', 5, 20)} ${number('calMs', 500, 3000)}`))));
  $('forget').addEventListener('click', () => action(() => request(`web forget ${token()}`)));
  for (const [id, cmd] of [['home', 'home'], ['pause', 'pause'], ['resume', 'resume'], ['next', 'next']]) $(id).addEventListener('click', () => action(() => request(`web ${cmd} ${token()}`)));
  for (const [id, mode] of [['startAuto', 'auto'], ['startSteps', 'steps']]) $(id).addEventListener('click', () => action(async () => {await request(`web start ${token()} ${mode}`); selectTab('overview');}));
  $('editActive').addEventListener('click', () => selectTab('program'));
  $('loadProgram').addEventListener('click', () => action(() => openSlot(number('slot', 1, 4))));
  $('refreshProgram').addEventListener('click', () => action(async () => {persistDraft(); pushUndo(); forceReload = true; await request('web info');}));
  $('applyProgram').addEventListener('click', () => action(applyProgram));
  $('saveProgram').addEventListener('click', () => action(async () => {await applyProgram(); await request(`web program ${token()} save`); await request('web info'); note('Программа сохранена в Mega'); event('Программа сохранена в EEPROM');}));
  $('undoProgram').addEventListener('click', () => {const previous = undoStack.pop(); if (previous) fillEditor(previous, true);});
  $('resizeZones').addEventListener('click', () => mutateProgram(p => ProgramModel.resize(p, number('zoneCount', 1, 10))));
  $('addZone').addEventListener('click', () => mutateProgram(p => ProgramModel.resize(p, p.zones.length + 1)));
  $('duplicateZone').addEventListener('click', () => mutateProgram(p => ProgramModel.duplicate(p, selectedZone)));
  $('removeZone').addEventListener('click', () => mutateProgram(p => ProgramModel.remove(p, selectedZone)));
  $('onlySelectedZone').addEventListener('click',()=>mutateProgram(p=>{p.zones.forEach((z,i)=>z.enabled=i+1===selectedZone?1:0);return p;}));
  for (const [id, enabled] of [['enableAllZones', 1], ['disableAllZones', 0]]) $(id).addEventListener('click', () => mutateProgram(p => {p.zones.forEach(z => z.enabled = enabled); return p;}));
  for (const id of ['programName', 'home1', 'home2', 'travel1', 'travel2', 'drip', 'tiltPct', 'lowSide', 'dryEnabled', 'drySeconds', 'dryStaging', 'dryX1', 'dryX2', 'dryZ1', 'dryZ2', 'zoneHPct', 'zoneVPct']) {$(id).addEventListener('focus', pushUndo); $(id).addEventListener('change', () => {try {draft = collectProgram();} catch (e) {note(e.message, true);} dirty();});}
  $('applyZoneSpeedAll').addEventListener('click', () => mutateProgram(p => {const selected = p.zones[selectedZone - 1]; for (const z of p.zones) if (z.enabled) {z.hPct = selected.hPct; z.vPct = selected.vPct;} return p;}));
  document.querySelectorAll('[data-capture]').forEach(b => b.addEventListener('click', () => captureCoordinates(b.dataset.capture)));
  $('captureDry').addEventListener('click', () => captureCoordinates('dry')); $('captureZoneX').addEventListener('click', () => captureCoordinates('x')); $('captureZoneZ').addEventListener('click', () => captureCoordinates('z'));
  $('testPlan').addEventListener('click', () => {try {const position = connected() ? currentCoordinates() : [1000, 1000, 1000, 1000]; mutateProgram(p => ProgramModel.demo(p.slot, position, number('testDx', 0, 500), number('testDz', 0, 500), number('testWait', 0, 60))); note('Пример создан в форме. Передайте его в Mega перед запуском.');} catch (e) {note(e.message, true);}});
  $('exportProgram').addEventListener('click', () => {try {download('telfer-program.json', JSON.stringify(collectProgram(), null, 2), 'application/json');} catch (e) {note(e.message, true);}});
  $('importProgram').addEventListener('change', async e => {try {const imported = ProgramModel.normalize(JSON.parse(await e.target.files[0].text())); pushUndo(); fillEditor(imported, true); note('Импортировано в форму; управление не запущено.');} catch (e) {note(e.message, true);}});
  $('readParams').addEventListener('click', () => action(readParameters)); $('paramGroup').addEventListener('change', filterParameters);
  document.querySelectorAll('[data-write-param]').forEach(b => b.addEventListener('click', () => action(() => writeParameter(b))));
  document.querySelectorAll('[data-profile]').forEach(b => b.addEventListener('click', () => action(async () => {const card = b.closest('[data-profile-card]'), values = [...card.querySelectorAll('input')].map((x, i) => integer(x.value, i < 3 ? 5 : i === 3 ? 0 : 1, i < 3 ? 80 : i === 3 ? 3000 : 100)); await request(`web profile ${token()} ${b.dataset.profile} ${values.join(' ')}`); card.dataset.dirty = '0';})));
  $('saveSettings').addEventListener('click', () => action(() => request(`web settings-save ${token()}`)));
  document.querySelectorAll('[data-sensor]').forEach(b => b.addEventListener('click', () => action(async () => {const card = b.closest('[data-sensor-card]'); await request(`web sensor ${token()} ${b.dataset.sensor} ${+card.querySelector('[data-inverse]').checked} ${integer(card.querySelector('[data-offset]').value, -10000, 10000)}`); card.dataset.dirty = '0'; await request('web info');})));
  document.querySelectorAll('[data-profile-card] input,[data-sensor-card] input').forEach(input => input.addEventListener('change', () => input.closest('fieldset').dataset.dirty = '1'));
  $('simDrive').addEventListener('change', renderSimulation);
  $('simRate').addEventListener('change', () => action(() => request(`web sim ${controller.session} rate ${number('simRate', 1, 5)}`)));
  $('simEstop').addEventListener('change', () => action(() => request(`web sim ${controller.session} estop ${+$('simEstop').checked}`)));
  $('simStartup').addEventListener('change', () => action(async () => {await request(`web sim ${controller.session} startup ${+$('simStartup').checked}`); parameters.clear(); document.querySelectorAll('[data-param] input').forEach(input => input.value = ''); note('Пусковой сценарий изменён в модели. Перечитайте параметры.');}));
  document.querySelectorAll('[data-sim-fault]').forEach(input => input.addEventListener('change', () => action(() => request(`web sim ${controller.session} ${input.dataset.simFault} ${$('simDrive').value} ${+input.checked}`))));
  $('simReset').addEventListener('click', () => action(async () => {await request(`web sim ${controller.session} reset`); parameters.clear(); $('simStartup').checked = false; note('Модель сброшена. Программа сохранена; повторный запуск требует допуска.');}));
  $('simSetPosition').addEventListener('click', () => action(() => request(`web sim ${controller.session} position ${['simX1', 'simX2', 'simZ1', 'simZ2'].map(id => number(id, 0, 10000)).join(' ')}`)));
  $('saveLog').addEventListener('click', () => download(`telfer-step9l-${new Date().toISOString().replace(/[:.]/g, '-')}.txt`, logLines.join('\n')));
  $('log').closest('details').addEventListener('toggle', () => {if ($('log').closest('details').open) $('log').textContent = logLines.slice(-1500).join('\n');});
  window.addEventListener('blur', () => {if (jog) releaseJog();}); window.addEventListener('pagehide', () => {persistDraft(); if (connected()) stop();});
  document.addEventListener('visibilitychange', () => {if (document.hidden && connected()) stop();}); window.addEventListener('keydown', e => {if (e.code === 'Escape' && connected()) stop();});
}
buildControls(); bindControls(); const restoredDraft=readDraft(1);fillEditor(restoredDraft || draft, !!restoredDraft); filterParameters(); renderAxes(); renderEvents(); render();
setInterval(() => {
  if (writer && reading && !supported && Date.now() - lastHandshake > 1000) {lastHandshake = Date.now(); write('service info').catch(() => {});}
  if (armed() && [...requests.values()].every(r => r.response === '@WEB_PARAM ')) write(`web ping ${controller.session}`).catch(() => {});
  if (controller.armed === '1' && Date.now() - lastRx > 2200) {localStop = true; note('Связь с Mega потеряна. Контроллер останавливает движение по таймеру.', true); controller.armed = '0'; releaseJog();}
  render(); renderAxes();
}, 250);
