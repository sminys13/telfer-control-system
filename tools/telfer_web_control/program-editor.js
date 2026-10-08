'use strict';
function storageKey(slot) {return `telfer.step9l.${source}.draft.${slot}`;}
function persistDraft() {try {localStorage.setItem(storageKey(draft.slot), JSON.stringify(collectProgram()));} catch {}}
function readDraft(slot) {try {const data = localStorage.getItem(storageKey(slot)); return data ? ProgramModel.normalize(JSON.parse(data)) : null;} catch {return null;}}
function pushUndo() {try {undoStack.push(collectProgram()); if (undoStack.length > 20) undoStack.shift();} catch {}}
function dirty() {formDirty = true; persistDraft(); renderProgramSummary(); render();}
function fillEditor(program, dirtyFlag = true) {
  draft = ProgramModel.normalize(program); formDirty = dirtyFlag; selectedZone = Math.min(Math.max(1, selectedZone), draft.zones.length);
  for (const id of ['home1', 'home2', 'travel1', 'travel2', 'drip', 'tiltPct', 'lowSide']) $(id).value = draft[id];
  $('slot').value = draft.slot; $('programName').value = draft.name; $('zoneCount').value = draft.zones.length;
  $('linkSides').checked = draft.zones.every(z => z.x1 === z.x2 && z.z1 === z.z2);
  $('dryEnabled').checked = draft.drying.enabled === 1;
  for (const [id, key] of [['drySeconds', 'seconds'], ['dryStaging', 'staging'], ['dryX1', 'x1'], ['dryX2', 'x2'], ['dryZ1', 'z1'], ['dryZ2', 'z2']]) $(id).value = draft.drying[key];
  renderZones(); renderSlots(); render(); if (dirtyFlag) persistDraft();
}
function collectProgram() {
  const data = ProgramModel.copy(draft);
  data.name = $('programName').value; data.slot = draft.slot;
  for (const id of ['home1', 'home2', 'travel1', 'travel2', 'drip', 'tiltPct', 'lowSide']) data[id] = $(id).value;
  for (const row of $('zones').children) {
    const z = data.zones[Number(row.dataset.zone) - 1];
    for (const input of row.querySelectorAll('[data-key]')) z[input.dataset.key] = input.type === 'checkbox' ? +input.checked : input.value;
  }
  if (data.zones[selectedZone - 1]) {data.zones[selectedZone - 1].hPct = $('zoneHPct').value; data.zones[selectedZone - 1].vPct = $('zoneVPct').value;}
  data.drying = {enabled: +$('dryEnabled').checked, seconds: $('drySeconds').value, staging: $('dryStaging').value, x1: $('dryX1').value, x2: $('dryX2').value, z1: $('dryZ1').value, z2: $('dryZ2').value};
  return ProgramModel.normalize(data);
}
function mutateProgram(fn) {try {const previous = collectProgram(); undoStack.push(previous); if (undoStack.length > 20) undoStack.shift(); fillEditor(fn(ProgramModel.copy(previous)), true);} catch (e) {note(e.message, true);}}
function renderSelectedZone() {const z = draft.zones[selectedZone - 1]; if (!z) return; $('selectedZoneTitle').textContent = `Зона ${selectedZone} · скорости и координаты`; $('zoneHPct').value = z.hPct; $('zoneVPct').value = z.vPct;}
function renderZones() {
  const body = $('zones'); body.replaceChildren();
  for (const [ordinal, n] of draft.order.entries()) {
    const z = draft.zones[n - 1], row = document.createElement('tr'); row.dataset.zone = n;
    row.classList.toggle('selected', n === selectedZone); row.classList.toggle('disabled', !z.enabled);
    const field = key => `<input aria-label="Зона ${n}: ${key}" data-key="${key}" type="number" value="${z[key]}" min="0" max="${key === 'dip' || key === 'wait' ? 36000 : key === 'tilt' ? 5000 : 10000}">`;
    row.innerHTML = `<td class="orderCell">${ordinal + 1}<button data-move="-1" title="Выше" ${ordinal === 0 ? 'disabled' : ''}>↑</button><button data-move="1" title="Ниже" ${ordinal === draft.order.length - 1 ? 'disabled' : ''}>↓</button></td><td><label><input aria-label="Включить зону ${n}" data-key="enabled" type="checkbox" ${z.enabled ? 'checked' : ''}><button class="selectZone" data-select>Зона ${n}</button></label></td><td><div class="pair">${field('x1')}${field('x2')}</div></td><td><div class="pair">${field('z1')}${field('z2')}</div></td><td>${field('dip')}</td><td>${field('tilt')}</td><td>${field('wait')}</td>`;
    body.appendChild(row);
    row.querySelector('[data-select]').addEventListener('click', () => selectZone(n));
    row.querySelectorAll('[data-move]').forEach(b => b.addEventListener('click', () => {selectedZone = n; mutateProgram(p => ProgramModel.move(p, n, Number(b.dataset.move)));}));
    row.querySelectorAll('[data-key]').forEach(input => {
      input.addEventListener('focus', pushUndo);
      input.addEventListener('change', () => {
        const key = input.dataset.key;
        if ($('linkSides').checked && /^[xz][12]$/.test(key)) row.querySelector(`[data-key="${key[0]}${key[1] === '1' ? '2' : '1'}"]`).value = input.value;
        try {draft = collectProgram(); row.classList.toggle('disabled', !draft.zones[n - 1].enabled);} catch (e) {note(e.message, true);} dirty();
      });
    });
  }
  renderSelectedZone(); renderProgramSummary();
}
function selectZone(n) {try {draft = collectProgram();} catch (e) {note(e.message, true); return;} selectedZone = n; $('zones').querySelectorAll('tr').forEach(row => row.classList.toggle('selected', Number(row.dataset.zone) === n)); renderSelectedZone();}
function renderProgramSummary() {try {const p = collectProgram(); $('programSummary').textContent = `${p.zones.filter(z => z.enabled).length} из ${p.zones.length} включены · выдержки ${formatTime(ProgramModel.timed(p))}`;} catch (e) {$('programSummary').textContent = e.message;}}
function renderSlots() {
  const root = $('slotCards'); root.replaceChildren();
  for (let i = 1; i <= 4; i++) {
    const b = document.createElement('button'); b.dataset.slot = i; b.classList.toggle('active', i === Number($('slot').value));
    const title = document.createElement('strong'); title.textContent = slotNames[i - 1] || `Программа ${i}`;
    const hint = document.createElement('small'); hint.textContent = Number(controller.slot) === i ? 'Загружена в Mega' : `Слот ${i}`; b.append(title, hint);
    b.addEventListener('click', () => {persistDraft(); $('slot').value = i; action(() => openSlot(i));}); root.appendChild(b);
  }
}
async function openSlot(slot) {
  persistDraft(); const stored = readDraft(slot);
  if (armed() && !moving()) {forceReload = true; await request(`web program ${token()} slot ${slot}`); await request('web info'); if (stored) {fillEditor(stored, true); note('Открыт сохранённый черновик этого слота; примените перед запуском.');}}
  else {fillEditor(stored || {...ProgramModel.demo(slot), name: slotNames[slot - 1] || `Программа ${slot}`}, true); note('Черновик открыт локально. Передачу можно выполнить после подключения.');}
}
async function applyProgram() {
  const d = ProgramModel.ready(collectProgram()), session = token();
  forceKeepDraft = true;
  try {
    if (Number(controller.slot) !== d.slot) {await request(`web program ${session} slot ${d.slot}`); await request('web info');}
    await request(`web program ${session} count ${d.zones.length}`);
    await request(`web program ${session} home ${d.home1} ${d.home2}`);
    await request(`web program ${session} travel ${d.travel1} ${d.travel2}`);
    await request(`web program ${session} recipe ${d.drip} ${d.tiltPct} ${d.lowSide}`);
    await request(`web program ${session} name ${encodeName(d.name)}`);
    for (const z of d.zones) await request(`web zone ${session} ${z.n} ${z.x1} ${z.x2} ${z.z1} ${z.z2} ${z.dip} ${z.tilt} ${z.wait} ${z.hPct} ${z.vPct} ${z.enabled}`);
    const dry = d.drying;
    await request(`web program ${session} dry ${dry.enabled} ${dry.seconds} ${dry.staging} ${dry.x1} ${dry.x2} ${dry.z1} ${dry.z2}`);
    await request(`web program ${session} order ${d.order.join(' ')}`);
    await request(`web program ${session} commit`);
    formDirty = false; forceKeepDraft = false; await request('web info'); note('Программа применена в текущем сеансе Mega. Запуск доступен на «Обзоре».'); event(`Программа ${d.slot} применена`);
    try {localStorage.removeItem(storageKey(d.slot));} catch {}
  } finally {forceKeepDraft = false;}
}
function currentCoordinates() {if (!connected() || !lastRx || Date.now() - lastRx > 2200 || axes.some(a => !a || a.valid === '0' || a.hwOk === '0' || Number(a.age) > 1200)) throw Error('Нужны четыре свежих координаты'); return axes.map(a => integer(a.mm, 0, 10000));}
function captureCoordinates(target) {try {const mm = currentCoordinates(); mutateProgram(p => {if (target === 'home') {p.home1 = mm[0]; p.home2 = mm[1];} else if (target === 'travel') {p.travel1 = mm[2]; p.travel2 = mm[3];} else if (target === 'dry') {p.drying.x1 = mm[0]; p.drying.x2 = mm[1]; p.drying.z1 = mm[2]; p.drying.z2 = mm[3];} else {const z = p.zones[selectedZone - 1]; if (target === 'x') {z.x1 = mm[0]; z.x2 = mm[1];} else {z.z1 = mm[2]; z.z2 = mm[3];}} return p;});} catch (e) {note(e.message, true);}}
