'use strict';
function formatTime(seconds) {if (!Number.isFinite(Number(seconds)) || Number(seconds) === 65535) return '—'; const s = Math.max(0, Number(seconds)); return s >= 3600 ? `${Math.floor(s / 3600)} ч ${Math.floor(s % 3600 / 60)} мин` : s >= 60 ? `${Math.floor(s / 60)} мин ${s % 60} с` : `${s} с`;}
function renderEvents() {for (const [id, count] of [['recentEvents', 4], ['events', 200]]) {const root = $(id); root.replaceChildren(); for (const e of events.slice(0, count)) {const li = document.createElement('li'); li.classList.toggle('error', e.error); const time = document.createElement('time'); time.textContent = e.at; li.append(time, document.createTextNode(e.text)); root.appendChild(li);} if (!events.length) {const li = document.createElement('li'); li.textContent = 'Событий ещё нет'; root.appendChild(li);}}}
function renderAxes() {
  names.forEach((name, i) => {
    const a = axes[i] || {}, card = $('axes').children[i]; if (!card) return;
    const stale = a.age === undefined || Number(a.age) > 1200 || a.valid === '0' || a.hwOk === '0' || !lastRx || Date.now() - lastRx > 2200;
    card.classList.toggle('stale', stale); card.querySelector('.value').textContent = a.mm === undefined ? '—' : `${a.mm} мм`;
    card.querySelector('.state').textContent = stale ? 'Данные отсутствуют или устарели' : `${isSim() ? 'Модель' : 'Датчик'} · ${a.age} мс`;
    card.querySelector('.hz').textContent = `выход ${a.runHz === undefined ? '—' : a.runHz.toFixed(2)} · задание ${a.setHz === undefined ? '—' : a.setHz.toFixed(2)} Гц`;
    card.querySelector('.fault').textContent = a.online === '0' ? 'Нет связи с ПЧ' : a.fault && !['0', '0x0'].includes(a.fault) ? 'Ошибка ПЧ ' + a.fault : '';
    const c = $('calCards').children[i]; if (c) c.querySelector('.result').textContent = maskReady(1 << i) ? `${isSim() ? 'Задано/проверено в модели' : 'Измерено'} · FWD ${Number(a.fwdSign) > 0 ? 'увеличивает' : 'уменьшает'} расстояние` : 'Нужны измерения обоих направлений';
  });
  renderMap();
}
function render() {
  $('connect').disabled = !!port; $('disconnect').disabled = !port; $('stop').disabled = !connected();
  $('connection').textContent = port ? 'USB · Mega' : 'Не подключён';
  $('modeBadge').textContent = connected() ? isSim() ? 'ИМИТАЦИЯ · Mega' : 'ОБОРУДОВАНИЕ · FIELD' : 'Нет подключения'; $('modeBadge').classList.toggle('field', connected() && !isSim());
  $('sourceHint').textContent = isSim() ? 'Датчики и ПЧ виртуальные. Физические выходы отключены.' : 'Редактор отделён от запуска. Рабочее управление требует аппаратной защиты.';
  $('externalLabel').hidden = isSim(); $('simulationPanel').hidden = !connected() || !isSim();
  $('permit').textContent = armed() ? 'Управление разрешено' : 'Управление отключено'; $('permit').className = armed() ? 'good' : '';
  $('arm').disabled = !connected() || !controller.session || (!isSim() && !$('external').checked) || armed() || busy || controller.busBusy === '1';
  $('arm').textContent = isSim() ? 'Начать имитацию' : 'Разрешить управление';
  $('clearSafety').disabled = !connected() || Number(controller.estop) !== 2 || busy || moving();
  $('safety').textContent = `${isSim() ? 'E-STOP модели' : 'Вход E-STOP Mega'}: ${Number(controller.estop) ? 'активен / защёлкнут' : connected() ? 'OK' : '—'}${isSim() ? ' · физическое управление отключено' : ' · концевики Mega: ' + (controller.limitsEnabled === '1' ? 'включены' : 'аппаратная цепь')}`;
  $('calHint').textContent = isSim() ? 'Начальные знаки задаёт модель. Импульсы проверяют виртуальные измерения и не калибруют реальную установку.' : 'Рабочие направления подтверждаются лазерами в обоих направлениях; парные движения требуют подтверждений обоих приводов.';
  document.querySelectorAll('[data-jog]').forEach(b => b.disabled = !armed() || !maskReady(Number(b.dataset.mask)) || busy || controller.running === '1' || controller.pulse === '1');
  document.querySelectorAll('[data-calibrate]').forEach(b => b.disabled = !armed() || busy || moving());
  for (const id of ['forget', 'saveProgram', 'applyProgram', 'saveSettings']) $(id).disabled = !armed() || busy || moving();
  $('loadProgram').disabled = busy || moving(); $('refreshProgram').disabled = !connected() || busy || moving(); $('readParams').disabled = !connected() || busy || moving();
  $('home').disabled = !armed() || !maskReady(15) || busy || moving() || controller.editing === '1';
  for (const id of ['startAuto', 'startSteps']) $(id).disabled = !armed() || !maskReady(15) || controller.ready !== '1' || busy || moving() || controller.editing === '1' || (formDirty && Number(controller.slot) === draft.slot);
  $('pause').disabled = !armed() || controller.running !== '1' || controller.paused === '1'; $('resume').disabled = !armed() || controller.paused !== '1'; $('next').disabled = !armed() || controller.running !== '1';
  $('runState').textContent = controller.running === '1' ? controller.paused === '1' ? 'Пауза' : 'Программа выполняется' : controller.phase === 'DONE' ? 'Цикл завершён' : Number(controller.error) || Number(controller.issue) ? 'Останов с причиной' : 'Остановлено';
  $('phaseTitle').textContent = phases[controller.phase] || 'Готовность к запуску'; $('stepDetail').textContent = `Шаг ${controller.step || 0} / ${controller.steps || 0} · зона ${controller.zone || '—'}`;
  $('elapsed').textContent = `Прошло ${formatTime(timeState.elapsed)}`; $('waitTime').textContent = `Ожидание ${formatTime(controller.wait)}`;
  $('cycleHint').textContent = controller.waitOperator === '1' ? 'Выполните действие и нажмите «Следующий шаг»' : controller.paused === '1' ? 'Можно продолжить с текущего этапа' : controller.ready === '1' ? 'Программа готова; запуск выполняется кнопкой' : 'Передайте готовую программу из редактора';
  $('activeProgram').textContent = runtimeProgram ? `${runtimeProgram.name} · слот ${runtimeProgram.slot}${controller.programDirty === '1' ? ' · изменения в RAM' : ' · сохранена'}${isSim() ? ' · учебная память' : ''}` : 'Программа Mega ещё не прочитана';
  $('progress').firstElementChild.style.width = (Number(controller.steps) ? Math.min(100, Number(controller.step) / Number(controller.steps) * 100) : 0) + '%';
  $('draftStatus').textContent = !connected() ? 'Локальный черновик · Mega не подключена' : formDirty ? 'Черновик изменён · ещё не передан' : controller.programDirty === '1' ? 'Применено в RAM · сохраните для следующего включения' : 'Совпадает с программой Mega';
  document.querySelectorAll('[data-profile],[data-sensor]').forEach(b => b.disabled = !armed() || busy || moving());
  document.querySelectorAll('[data-write-param]').forEach(b => {const cell=b.closest('[data-param]'),def=paramDefs.find(p=>p[0]===cell.dataset.param);b.disabled=!armed()||busy||moving()||parameters.get(`${cell.dataset.drive}:${cell.dataset.param}`)?.ok!=='1'||!scale(def,cell.dataset.drive);});
  document.querySelectorAll('[data-slot]').forEach(b => b.disabled = busy || moving());
  $('undoProgram').disabled = !undoStack.length;
  $('simReset').disabled = !connected() || !isSim() || busy; $('simSetPosition').disabled = !connected() || !isSim() || busy || moving(); $('simStartup').disabled = !connected() || !isSim() || busy || moving();
  renderRoute();
}
function renderRoute() {
  const root=$('route');if(!runtimeProgram){root.replaceChildren();renderRoute.key='';return;}
  const key=JSON.stringify([runtimeProgram.slot,runtimeProgram.order,runtimeProgram.zones.map(z=>z.enabled)]);
  if(renderRoute.key!==key){
    root.replaceChildren();renderRoute.key=key;
    for(const n of runtimeProgram.order){const b=document.createElement('button');b.dataset.zone=n;b.textContent=`Зона ${n}${runtimeProgram.zones[n-1].enabled?'':' · пропуск'}`;b.classList.toggle('skipped',!runtimeProgram.zones[n-1].enabled);b.addEventListener('click',()=>{selectTab('program');if(draft.slot===runtimeProgram.slot&&draft.zones[n-1])selectZone(n);});root.appendChild(b);}
    root.appendChild(document.createTextNode(' → HOME'));
  }
  root.querySelectorAll('button').forEach(b=>b.classList.toggle('current',controller.running==='1'&&Number(controller.zone)===Number(b.dataset.zone)));
}
function renderMap() {
  const svg = $('plantMap'); if (!svg || !runtimeProgram) {if (svg) svg.replaceChildren(); return;}
  const p = runtimeProgram, points = p.zones.map(z => (z.x1 + z.x2) / 2), current = axes[0]?.mm !== undefined && axes[1]?.mm !== undefined ? (Number(axes[0].mm) + Number(axes[1].mm)) / 2 : (p.home1 + p.home2) / 2;
  const min = Math.min(...points, p.home1, p.home2, current) - 40, max = Math.max(...points, p.home1, p.home2, current) + 40, scale = x => 60 + (x - min) / Math.max(80, max - min) * 780;
  let html = '<line x1="50" y1="72" x2="850" y2="72" stroke="#61758c" stroke-width="3"/>';
  for (let i = 0; i < points.length; i++) {const x = scale(points[i]), active = controller.running === '1' && Number(controller.zone) === i + 1; html += `<rect x="${x - 22}" y="105" width="44" height="28" rx="5" fill="${active ? '#327660' : '#2f475f'}"/><text x="${x}" y="156" fill="#bbccdc" text-anchor="middle" font-size="15">Зона ${i + 1}</text>`;}
  html += `<line x1="${scale(current)}" y1="65" x2="${scale(current)}" y2="108" stroke="#71bbf3" stroke-width="3"/><rect x="${scale(current) - 25}" y="48" width="50" height="28" rx="5" fill="#4693c4"/><text x="${scale(current)}" y="32" fill="#daeefd" text-anchor="middle" font-size="17">X ${Math.round(current)} мм</text>`;
  svg.innerHTML = html;
}
function renderSimulation() {if (!isSim()) return; $('simRate').value = simulation.rate || '1'; $('simStartup').checked=simulation.startup==='1'; $('simEstop').checked = (Number(controller.estop) & 1) !== 0; const bit = 1 << names.indexOf($('simDrive').value); document.querySelectorAll('[data-sim-fault]').forEach(input => input.checked = (Number(simulation[input.dataset.simFault === 'sensor' ? 'sensorLost' : input.dataset.simFault]) & bit) !== 0);}
function sampleTrend() {if (axes.some(a => !a || a.valid === '0' || Number(a.age) > 1200 || !Number.isFinite(Number(a.mm)))) return; const now = Date.now(); if (trend.length && now - trend.at(-1).at < 450) return; trend.push({at: now, values: axes.map(a => Number(a.mm))}); while (trend.length && now - trend[0].at > 60000) trend.shift(); drawTrend();}
function drawTrend() {const canvas = $('trend'), ctx = canvas.getContext('2d'); ctx.clearRect(0, 0, canvas.width, canvas.height); if (trend.length < 2) return; const values = trend.flatMap(p => p.values), min = Math.min(...values) - 10, max = Math.max(...values) + 10, end = trend.at(-1).at; ctx.font = '13px system-ui'; ctx.fillStyle = '#a1b2c5'; ctx.fillText(`${max} мм`, 5, 16); ctx.fillText(`${min} мм`, 5, 166); ['#69b8ef', '#b2e5ff', '#77d9b3', '#efcf75'].forEach((color, i) => {ctx.strokeStyle = color; ctx.beginPath(); trend.forEach((p, j) => {const x = 65 + (p.at - end + 60000) / 60000 * 820, y = 160 - (p.values[i] - min) / (max - min) * 140; if (!j) ctx.moveTo(x, y); else ctx.lineTo(x, y);}); ctx.stroke();});}
