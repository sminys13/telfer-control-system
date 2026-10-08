'use strict';

let port = null;
let reader = null;
let writer = null;
let reading = false;
let receiveBuffer = '';
let logLines = [];
let firmware = '';
let buildMode = 'unknown'; // bench | readonly | field
let sourceConfirmed = false;

const driveNames = ['H1','H2','V1','V2'];
const driveBits = {H1:1,H2:2,V1:4,V2:8};
const state = {
  gates: {preflight:false, preflightResult:'wait', preflightReason:'', protocol:0, pulsePos:0, pulseNeg:0, confirmPos:0, confirmNeg:0, pair:0, assist:'OFF'},
  protocol: {H1:null,H2:null,V1:null,V2:null},
  pulse: {},
  assist: {axis:'X',state:'OFF',pct:0,ageA:null,ageB:null,freshPairs:0},
  program: null,
  zones: new Map(),
};

const $ = (id) => document.getElementById(id);
const logEl = $('log');
const encoder = new TextEncoder();

function ts(){const d=new Date();return d.toLocaleTimeString('ru-RU',{hour12:false})+'.'+String(d.getMilliseconds()).padStart(3,'0');}
function sleep(ms){return new Promise(r=>setTimeout(r,ms));}
function appendLog(text,kind='rx'){
  const prefix=kind==='tx'?'>>':kind==='local'?'##':'  ';
  const line=`[${ts()}] ${prefix} ${text}`;
  logLines.push(line); if(logLines.length>60000) logLines.splice(0,5000);
  const span=document.createElement('span');
  const pass=/\bPASS\b|CONFIRMED|ONLINE|HEALTHY/i.test(text);
  const bad=/\bFAIL\b|FAULT|rejected|TIMEOUT|error=[1-9]|BLOCKED/i.test(text);
  span.className=kind==='tx'?'tx':kind==='local'?'local':bad?'error-line':pass?'pass-line':'';
  span.textContent=line+'\n'; logEl.appendChild(span);
  if($('autoScroll')?.checked) logEl.scrollTop=logEl.scrollHeight;
}
function alertBox(text,bad=false){const el=$('runtimeAlert');if(!text){el.classList.add('hidden');return;}el.classList.remove('hidden');el.textContent=text;el.style.borderColor=bad?'var(--bad)':'var(--warn)';}
function setConnected(on){$('connectBtn').disabled=on;$('disconnectBtn').disabled=!on;$('stopAllBtn').disabled=!on;const p=$('connectionPill');p.textContent=on?'ПОДКЛЮЧЕНО':'ОТКЛЮЧЕНО';p.className=`pill ${on?'online':'offline'}`;updateControls();updateDecision();}
function parseKV(line){const o={};for(const t of line.trim().split(/\s+/).slice(1)){const i=t.indexOf('=');if(i>0)o[t.slice(0,i)]=t.slice(i+1);}return o;}
function num(v,def=0){if(v===undefined||v===null)return def;const s=String(v);const n=/^0x/i.test(s)?parseInt(s,16):parseInt(s,10);return Number.isFinite(n)?n:def;}
function hexMask(v){if(v===undefined)return 0;const s=String(v);return parseInt(s.replace(/^0x/i,''),16)||0;}
function setCell(row,cls,val,klass=''){const c=row?.querySelector('.'+cls);if(!c)return;c.textContent=val??'—';c.className=cls+(klass?' '+klass:'');}
function gateEl(name){return document.querySelector(`.gate[data-gate="${name}"]`);}
function setGate(name,text,status='wait'){const g=gateEl(name);if(!g)return;g.classList.remove('pass','fail','wait');g.classList.add(status);const s=g.querySelector('span');if(s)s.textContent=text;}
function popcount4(x){x&=15;let c=0;for(;x;x>>=1)c+=x&1;return c;}
function directionsCount(){return popcount4(state.gates.confirmPos)+popcount4(state.gates.confirmNeg);}
function preflightReasonText(reason){
  const map={
    ESTOP:'E-STOP активен или его защёлка не сброшена',
    LIMIT_INPUT_ACTIVE:'активен один или несколько входов концевиков',
    DRIVE_STATUS_NOT_FRESH:'нет свежей телеметрии одного из HE200',
    DRIVE_FAULT:'один из HE200 сообщает аварию',
    DRIVE_NOT_STOPPED:'один из HE200 не подтверждает останов',
    FIELD_SERVICE_BUILD_REQUIRED:'загружена не FIELD SERVICE сборка'
  };
  return map[reason]||reason||'причина не указана';
}
function updateDecision(){
  const el=$('gateDecision');if(!el)return;
  let cls='wait',msg='Подключите COM и загрузите FIELD SERVICE.';
  if(port&&buildMode!=='field'){cls='fail';msg='NO-GO: для физических проверок нужна сборка mega_v6_he200_field_service.';}
  else if(port&&buildMode==='field'&&state.gates.preflightResult==='fail'){
    cls='fail';msg=`PRE-FLIGHT FAIL: ${preflightReasonText(state.gates.preflightReason)}. Исправьте условие и повторите «Подготовить систему».`;
  }
  else if(port&&buildMode==='field'&&!state.gates.preflight){cls='warn';msg='Этап 1: нажмите «Подготовить систему». Дальше только после PRE-FLIGHT PASS.';}
  else if(state.gates.preflight&&(state.gates.protocol&15)!==15){
    const anyFail=Object.values(state.protocol).some(v=>v===false);cls=anyFail?'fail':'warn';
    msg=anyFail?'NO-GO: один из protocol probe завершился FAIL. Сохраните журнал и не переключайте источник управления.':'PRE-FLIGHT PASS. Этап 2: можно проверить Modbus-карту всех четырёх HE200.';
  } else if(protocolReady()&&!sourceConfirmed){cls='pass';msg='PROTOCOL 4/4 PASS. Можно перейти к ручной установке P0.02=2 и P0.03=9 на четырёх HE200, затем поставить галочку подтверждения.';}
  else if(protocolReady()&&sourceConfirmed&&directionsCount()<8){cls='pass';msg='Протокол и источник подтверждены. Можно выполнять низкоскоростные импульсы и подтверждать направления до 8/8.';}
  else if(fullDirectionReady()&&popcount4(state.gates.pair&3)<2){cls='pass';msg='Направления 8/8. Можно выполнять парные тесты H и Z до 2/2.';}
  else if(fullDirectionReady()&&popcount4(state.gates.pair&3)===2){cls='pass';msg='ВСЕ ВОРОТА ПУСКОНАЛАДКИ PASS. Разрешён контролируемый Assisted motion выбранной пары; AUTO/HOME всё ещё заблокированы.';}
  el.className='decision '+cls;el.querySelector('span').textContent=msg;
}
function updateGateUI(){
  if(state.gates.preflightResult==='fail') setGate('preflight',`FAIL: ${state.gates.preflightReason||'?'}`,'fail');
  else if(state.gates.preflight) setGate('preflight','PASS','pass');
  else setGate('preflight','ОЖИДАНИЕ','wait');
  const pc=popcount4(state.gates.protocol);setGate('protocol',`${pc}/4`,pc===4?'pass':pc?'wait':'wait');
  setGate('source',sourceConfirmed?'ПОДТВЕРЖДЁН':'НЕ ПОДТВЕРЖДЁН',sourceConfirmed?'pass':'wait');
  const dc=directionsCount();setGate('directions',`${dc}/8`,dc===8?'pass':'wait');
  const pairs=popcount4(state.gates.pair&3);setGate('pairs',`${pairs}/2`,pairs===2?'pass':'wait');
  const as=state.gates.assist||'OFF';setGate('assist',as,as==='FAULT'?'fail':as==='NORMAL'||as==='RECOVER'||as==='DECEL'||as==='HOLD'||as==='WARNING'||as==='STARTING'?'pass':'wait');
  $('assistState').textContent=as;$('assistState').style.color=as==='FAULT'?'var(--bad)':as==='DECEL'||as==='HOLD'||as==='WARNING'?'var(--warn)':'var(--good)';
  updateProtocolCards();updatePulseCards();updateControls();updateDecision();
}
function fieldReady(){return buildMode==='field' && !!port;}
function commissioningReady(){return fieldReady() && state.gates.preflight;}
function protocolReady(){return commissioningReady() && (state.gates.protocol&15)===15;}
function fullDirectionReady(){return protocolReady() && sourceConfirmed && (state.gates.confirmPos&15)===15 && (state.gates.confirmNeg&15)===15;}
function updateControls(){
  const connected=!!port;
  $('prepareBtn').disabled=!connected || buildMode!=='field';
  $('probeAllBtn').disabled=!protocolReady() && !(commissioningReady());
  if($('sourceConfirm')) $('sourceConfirm').disabled=!protocolReady();
  document.querySelectorAll('[data-pair]').forEach(b=>{b.disabled=!fullDirectionReady();});
  const pairBit=$('assistAxis')?.value==='z'?2:1;
  const assistOk=fullDirectionReady() && !!(state.gates.pair&pairBit) && state.gates.assist!=='FAULT';
  $('assistPosBtn').disabled=!assistOk;$('assistNegBtn').disabled=!assistOk;
  $('assistStopBtn').disabled=!connected;$('assistClearBtn').disabled=!fieldReady();
  $('refreshDrivesBtn').disabled=!connected;
  ['sensorProfileBtn','sensorReinitBtn','sensorStatusBtn','sensorResetBtn','programLoadBtn','programSaveBtn','programRefreshBtn','setHomeBtn','capHomeBtn','setTravelBtn','capTravelBtn','setZoneXBtn','capZoneXBtn','setZoneZBtn','capZoneZBtn','setZoneTimesBtn','setZoneSpeedBtn','zoneOnBtn','zoneOffBtn'].forEach(id=>{if($(id))$(id).disabled=!connected;});
}
function buildProtocolCards(){const root=$('protocolCards');root.innerHTML='';for(const d of driveNames){const c=document.createElement('div');c.className='drive-card';c.dataset.protocol=d;c.innerHTML=`<h3>${d}</h3><div class="result">Ожидание общего теста</div>`;root.appendChild(c);}}
function updateProtocolCards(){for(const d of driveNames){const c=document.querySelector(`[data-protocol="${d}"]`);if(!c)continue;const r=c.querySelector('.result');const v=state.protocol[d];r.textContent=v===true?'PASS — 1000H/701CH + 2000H=0006':v===false?'FAIL — остановиться и смотреть журнал':'Ожидание';r.className='result '+(v===true?'cell-good':v===false?'cell-bad':'');}}
function pulseKey(d,dir){return `${d}:${dir}`;}
function buildPulseCards(){const root=$('pulseCards');root.innerHTML='';for(const d of driveNames){const c=document.createElement('div');c.className='drive-card';c.dataset.pulseDrive=d;c.innerHTML=`<h3>${d}</h3><div class="button-row"><button data-pulse-drive="${d}" data-dir="pos">Коротко +</button><button data-pulse-drive="${d}" data-dir="neg">Коротко −</button></div><div class="button-row"><button data-confirm-drive="${d}" data-dir="pos">✓ + направление верно</button><button data-confirm-drive="${d}" data-dir="neg">✓ − направление верно</button></div><div class="result">—</div>`;root.appendChild(c);}root.querySelectorAll('[data-pulse-drive]').forEach(b=>b.addEventListener('click',()=>sendPulse(b.dataset.pulseDrive,b.dataset.dir)));root.querySelectorAll('[data-confirm-drive]').forEach(b=>b.addEventListener('click',()=>confirmDirection(b.dataset.confirmDrive,b.dataset.dir)));updatePulseCards();}
function updatePulseCards(){for(const d of driveNames){const bit=driveBits[d],c=document.querySelector(`[data-pulse-drive="${d}"]`)?.closest('.drive-card');if(!c)continue;const posPass=!!(state.gates.pulsePos&bit),negPass=!!(state.gates.pulseNeg&bit),posConf=!!(state.gates.confirmPos&bit),negConf=!!(state.gates.confirmNeg&bit);c.querySelectorAll('[data-pulse-drive]').forEach(b=>b.disabled=!(protocolReady()&&sourceConfirmed));c.querySelector(`[data-confirm-drive="${d}"][data-dir="pos"]`).disabled=!posPass||posConf;c.querySelector(`[data-confirm-drive="${d}"][data-dir="neg"]`).disabled=!negPass||negConf;c.querySelector('.result').textContent=`Электр.: + ${posPass?'PASS':'—'} / − ${negPass?'PASS':'—'} · Визуально: + ${posConf?'OK':'—'} / − ${negConf?'OK':'—'}`;}}
function updateHe200(line){const v=parseKV(line),row=document.querySelector(`#he200Table tr[data-drive="${v.name}"]`);if(!row)return;const on=v.online==='1';setCell(row,'online',on?'ONLINE':`ERR ${v.error??''}`,on?'cell-good':'cell-bad');if(!on)return;setCell(row,'run',(num(v.run001)/100).toFixed(2));setCell(row,'set',(num(v.set001)/100).toFixed(2));setCell(row,'outi',(num(v.outI001)/100).toFixed(2));setCell(row,'di',v.di??'—');setCell(row,'fault',v.fault??'—',v.fault==='0x0'?'cell-good':'cell-bad');setCell(row,'state',v.state??'—');}
function updateLaser(line){const v=parseKV(line),row=document.querySelector(`#laserTable tr[data-laser="${v.name}"]`);if(!row)return;const hw=v.hw==='1',age=v.age==='NA'?null:num(v.age),ma=num(v.maxAge),uart=['uartOE','uartPE','uartFE','uartBI','uartFIFO'].reduce((a,k)=>a+num(v[k]),0);setCell(row,'hw',hw?'OK':'FAIL',hw?'cell-good':'cell-bad');setCell(row,'mm',v.mm??'—');setCell(row,'age',age===null?'NA':age,age!==null&&age>1200?'cell-bad':age!==null&&age>650?'cell-warn':'');setCell(row,'maxage',v.maxAge??'—',ma>5000?'cell-bad':ma>1200?'cell-warn':'');setCell(row,'rate',(num(v.rate10)/10).toFixed(1));setCell(row,'good',v.good??'—');setCell(row,'serr',`${v.sensorErr??'0'} (${v.errCode??'0'})`,num(v.sensorErr)?'cell-warn':'');setCell(row,'crc',v.crc??'0',num(v.crc)?'cell-bad':'');setCell(row,'uart',String(uart),uart?'cell-bad':'');const ack=hexMask(v.ack),missing=hexMask(v.missing),nack=hexMask(v.nack);setCell(row,'ack',`${v.ack??'—'}/${v.missing??'—'}/${v.nack??'—'}`,ack===0x3f&&missing===0&&nack===0?'cell-good':'cell-warn');}
function updateProgram(line){const v=parseKV(line);state.program=v;$('programSlot').value=v.slot||$('programSlot').value;$('homeX1').value=v.home1??'';$('homeX2').value=v.home2??'';$('travelZ1').value=v.travel1??'';$('travelZ2').value=v.travel2??'';const s=$('programStatus').querySelectorAll('b');if(s[0])s[0].textContent=v.slot??'—';if(s[1])s[1].textContent=v.ready==='1'?'ДА':'НЕТ';if(s[2])s[2].textContent=v.zones??'—';if(s[3])s[3].textContent=v.dirty==='1'?'ДА':'НЕТ';}
function updateZone(line){const v=parseKV(line);state.zones.set(num(v.n),v);if(num(v.n)!==num($('zoneN').value))return;$('zoneX1').value=v.x1??'';$('zoneX2').value=v.x2??'';$('zoneZ1').value=v.z1??'';$('zoneZ2').value=v.z2??'';$('zoneDip').value=v.dip??'';$('zoneWait').value=v.wait??'';$('zoneTilt').value=v.tilt??'';$('zoneHPct').value=v.hpct??'';$('zoneVPct').value=v.vpct??'';}
function updateAssist(line){const v=parseKV(line);state.assist.axis=v.axis||state.assist.axis;state.assist.state=v.state||state.assist.state;state.assist.pct=num(v.pct,state.assist.pct);state.assist.ageA=v.ageA!==undefined?num(v.ageA):state.assist.ageA;state.assist.ageB=v.ageB!==undefined?num(v.ageB):state.assist.ageB;state.assist.freshPairs=num(v.freshPairs,state.assist.freshPairs);state.gates.assist=state.assist.state;const b=$('assistDetails').querySelectorAll('b');if(b[0])b[0].textContent=state.assist.ageA??'—';if(b[1])b[1].textContent=state.assist.ageB??'—';if(b[2])b[2].textContent=`${state.assist.pct}%`;updateGateUI();}
function parseGateState(line){const v=parseKV(line);state.gates.preflight=v.preflight==='1';if(state.gates.preflight){state.gates.preflightResult='pass';state.gates.preflightReason='';}else if(state.gates.preflightResult!=='fail'){state.gates.preflightResult='wait';}state.gates.protocol=hexMask(v.protocol);state.gates.pulsePos=hexMask(v.pulsePos);state.gates.pulseNeg=hexMask(v.pulseNeg);state.gates.confirmPos=hexMask(v.confirmPos);state.gates.confirmNeg=hexMask(v.confirmNeg);state.gates.pair=hexMask(v.pair);state.gates.assist=v.assist||'OFF';updateGateUI();}
function parseGate(line){const v=parseKV(line);if(v.name==='PREFLIGHT'){state.gates.preflight=v.result==='PASS';state.gates.preflightResult=v.result==='PASS'?'pass':'fail';state.gates.preflightReason=v.result==='PASS'?'':(v.reason||'UNKNOWN');}if(v.name==='PROTOCOL'&&v.drive){state.protocol[v.drive]=v.result==='PASS';if(v.result==='PASS')state.gates.protocol|=driveBits[v.drive]||0;else state.gates.protocol&=~(driveBits[v.drive]||0);}if(v.name==='DIRECTION'&&v.drive){const bit=driveBits[v.drive]||0;if(v.dir==='POS'){if(v.result==='CONFIRMED')state.gates.confirmPos|=bit;else state.gates.confirmPos&=~bit;}else{if(v.result==='CONFIRMED')state.gates.confirmNeg|=bit;else state.gates.confirmNeg&=~bit;}}if(v.name==='ALL'&&v.result==='RESET'){state.gates={preflight:false,preflightResult:'wait',preflightReason:'',protocol:0,pulsePos:0,pulseNeg:0,confirmPos:0,confirmNeg:0,pair:0,assist:'OFF'};sourceConfirmed=false;$('sourceConfirm').checked=false;}updateGateUI();}
function parsePulse(line){const v=parseKV(line),mask=hexMask(v.mask),dir=v.dir==='POS'?'pos':'neg';if(v.state==='DONE'&&v.result==='PASS'){if(dir==='pos')state.gates.pulsePos|=mask;else state.gates.pulseNeg|=mask;if(mask===3)state.gates.pair|=1;if(mask===12)state.gates.pair|=2;}if(v.state==='DONE'&&v.result==='FAIL')appendLog(`Импульсный тест FAIL: ${v.reason||'см. журнал'}`,'local');updateGateUI();}
function detectBuild(line){if(line.startsWith('FW:')){firmware=line.slice(3).trim();$('firmwareLabel').textContent=firmware;}if(/Step9I service cockpit: native HE200/i.test(line)||/FIELD SERVICE/i.test(line)&&/WRITE/i.test(line)){buildMode='field';$('buildLabel').textContent='FIELD SERVICE';}else if(/HE200 physical RS485 READ-ONLY/i.test(line)){if(buildMode!=='field')buildMode='readonly';$('buildLabel').textContent='READ-ONLY';}else if(/physical MAX485 disabled|DRY-RUN/i.test(line)){if(buildMode==='unknown')buildMode='bench';$('buildLabel').textContent='BENCH/DRY';}updateControls();}
function processServiceInfo(line){const v=parseKV(line);firmware=v.fw||firmware;$('firmwareLabel').textContent=firmware||'—';buildMode=v.build==='FIELD'?'field':v.build==='READONLY'?'readonly':'bench';$('buildLabel').textContent=v.build||'—';if(v.auto==='1')alertBox('ВНИМАНИЕ: получена сборка с физическим AUTO. Для Step9I ожидается auto=0.',true);updateControls();}
function processLine(line){if(!line)return;$('lastRxLabel').textContent=new Date().toLocaleTimeString('ru-RU',{hour12:false});detectBuild(line);if(line.startsWith('@SERVICE_INFO '))processServiceInfo(line);else if(line.startsWith('@HE200 '))updateHe200(line);else if(line.startsWith('@LASER '))updateLaser(line);else if(line.startsWith('@GATE_STATE '))parseGateState(line);else if(line.startsWith('@GATE '))parseGate(line);else if(line.startsWith('@SERVICE_PULSE '))parsePulse(line);else if(line.startsWith('@ASSIST '))updateAssist(line);else if(line.startsWith('@PROGRAM '))updateProgram(line);else if(line.startsWith('@ZONE '))updateZone(line);}
async function readLoop(){const decoder=new TextDecoder();while(reading&&reader){try{const {value,done}=await reader.read();if(done)break;if(value){receiveBuffer+=decoder.decode(value,{stream:true});let i;while((i=receiveBuffer.indexOf('\n'))>=0){const line=receiveBuffer.slice(0,i).replace(/\r$/,'');receiveBuffer=receiveBuffer.slice(i+1);appendLog(line);processLine(line);}}}catch(e){appendLog('Ошибка чтения COM: '+e.message,'local');break;}}reading=false;}
async function connect(){if(!('serial'in navigator)){ $('browserWarning').classList.remove('hidden');return;}try{port=await navigator.serial.requestPort();await port.open({baudRate:115200,dataBits:8,stopBits:1,parity:'none',flowControl:'none',bufferSize:65536});writer=port.writable.getWriter();reader=port.readable.getReader();reading=true;setConnected(true);$('portLabel').textContent='115200 8N1';appendLog('COM-порт открыт на 115200 8N1','local');readLoop();await sleep(300);await send('service info');await sleep(100);await send('service gate status');await send('prog show');}catch(e){appendLog('Не удалось открыть COM: '+e.message,'local');await disconnect();}}
async function disconnect(){reading=false;try{if(reader){await reader.cancel();reader.releaseLock();}}catch{}try{if(writer)writer.releaseLock();}catch{}try{if(port)await port.close();}catch{}reader=null;writer=null;port=null;buildMode='unknown';setConnected(false);appendLog('COM-порт закрыт','local');}
async function send(cmd){if(!writer){appendLog('COM не подключён','local');return false;}appendLog(cmd,'tx');await writer.write(encoder.encode(cmd+'\n'));return true;}
async function prepare(){if(buildMode!=='field'){alertBox('Для ворот допуска и физических сервисных тестов загрузите mega_v6_he200_field_service. READ-ONLY можно использовать только для просмотра.',true);return;}state.gates.preflight=false;state.gates.preflightResult='wait';state.gates.preflightReason='';updateGateUI();alertBox('Подготовка: профиль связи → лазеры → телеметрия 4 ПЧ → PRE-FLIGHT. Все ПЧ должны стоять.');await send('log quiet');await sleep(120);await send('he200 comm');await sleep(180);await send('laser profile all');await sleep(1800);await send('test all');await sleep(1800);await send('service preflight');await sleep(350);await send('service gate status');await sleep(250);await send('laser status');await sleep(150);if(state.gates.preflight)alertBox('PRE-FLIGHT PASS. Можно переходить к проверке протокола.');else if(state.gates.preflightResult==='fail')alertBox(`PRE-FLIGHT FAIL: ${preflightReasonText(state.gates.preflightReason)}.`,true);else alertBox('PRE-FLIGHT: ответ ещё не получен. Нажмите «Подготовить систему» повторно или проверьте журнал.',true);}
async function probeAll(){if(!state.gates.preflight){alertBox('Сначала нужен PRE-FLIGHT PASS.',true);return;}sourceConfirmed=false;$('sourceConfirm').checked=false;await send('he200 probe all');await sleep(100);}
async function sendPulse(d,dir){if(!protocolReady()||!sourceConfirmed){alertBox('Нужны Protocol 4/4 и подтверждение P0.02=2 / P0.03=9.',true);return;}await send(`service pulse ${d.toLowerCase()} ${dir} 10 1200`);}
async function confirmDirection(d,dir){await send(`service confirm ${d.toLowerCase()} ${dir}`);await sleep(100);await send('service gate status');}
async function pairPulse(axis,dir){if(!fullDirectionReady()){alertBox('Сначала подтвердите оба направления всех четырёх приводов.',true);return;}await send(`service pulse ${axis} ${dir} 10 1200`);}
async function startAssist(dir){const axis=$('assistAxis').value;const pct=Math.max(5,Math.min(80,num($('assistPct').value,axis==='x'?40:68)));const pairBit=axis==='x'?1:2;if(!(state.gates.pair&pairBit)){alertBox('Сначала нужен PASS парного теста выбранной оси.',true);return;}await send(`assist ${axis} ${dir} ${pct}`);}
function getInt(id,min,max){const v=num($(id).value,NaN);if(!Number.isFinite(v)||v<min||v>max)throw new Error(`${id}: значение должно быть ${min}…${max}`);return v;}
async function safeProgram(fn){try{await fn();await sleep(120);await send('prog show');}catch(e){alertBox(e.message,true);}}
function saveLog(){const blob=new Blob([logLines.join('\n')+'\n'],{type:'text/plain;charset=utf-8'});const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=`telfer-step9i-${new Date().toISOString().replace(/[:.]/g,'-')}.txt`;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}

// Tabs
document.querySelectorAll('.tab').forEach(b=>b.addEventListener('click',()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.tabpage').forEach(x=>x.classList.remove('active'));b.classList.add('active');$(b.dataset.tab).classList.add('active');}));
buildProtocolCards();buildPulseCards();
$('connectBtn').addEventListener('click',connect);$('disconnectBtn').addEventListener('click',disconnect);$('stopAllBtn').addEventListener('click',()=>send('service stop'));
$('prepareBtn').addEventListener('click',prepare);$('probeAllBtn').addEventListener('click',probeAll);
$('sourceConfirm').addEventListener('change',e=>{if(e.target.checked&&!protocolReady()){e.target.checked=false;alertBox('Сначала Protocol HE200 должен быть 4/4 PASS.',true);return;}sourceConfirmed=e.target.checked;updateGateUI();appendLog(sourceConfirmed?'Оператор подтвердил P0.02=2 / P0.03=9 на четырёх ПЧ':'Подтверждение источника Modbus снято','local');});
document.querySelectorAll('[data-pair]').forEach(b=>b.addEventListener('click',()=>pairPulse(b.dataset.pair,b.dataset.dir)));
$('assistAxis').addEventListener('change',()=>{if($('assistAxis').value==='x'&&num($('assistPct').value)===68)$('assistPct').value=40;if($('assistAxis').value==='z'&&num($('assistPct').value)===40)$('assistPct').value=68;updateControls();});
$('assistPosBtn').addEventListener('click',()=>startAssist('pos'));$('assistNegBtn').addEventListener('click',()=>startAssist('neg'));$('assistStopBtn').addEventListener('click',()=>send('assist stop'));$('assistClearBtn').addEventListener('click',async()=>{await send('assist clear');await send('service gate status');});
$('refreshDrivesBtn').addEventListener('click',()=>send('test all'));
$('sensorProfileBtn').addEventListener('click',async()=>{await send('laser profile all');await sleep(1700);await send('laser status');});$('sensorReinitBtn').addEventListener('click',()=>send('laser reinit'));$('sensorStatusBtn').addEventListener('click',()=>send('laser status'));$('sensorResetBtn').addEventListener('click',()=>send('laser reset'));
$('programLoadBtn').addEventListener('click',()=>safeProgram(()=>send(`prog slot ${getInt('programSlot',1,4)}`)));$('programSaveBtn').addEventListener('click',()=>safeProgram(()=>send('prog save')));$('programRefreshBtn').addEventListener('click',()=>send('prog show'));
$('setHomeBtn').addEventListener('click',()=>safeProgram(()=>send(`set home ${getInt('homeX1',0,10000)} ${getInt('homeX2',0,10000)}`)));$('capHomeBtn').addEventListener('click',()=>safeProgram(()=>send('cap home')));
$('setTravelBtn').addEventListener('click',()=>safeProgram(()=>send(`set travel ${getInt('travelZ1',0,10000)} ${getInt('travelZ2',0,10000)}`)));$('capTravelBtn').addEventListener('click',()=>safeProgram(()=>send('cap travel')));
$('setZoneXBtn').addEventListener('click',()=>safeProgram(()=>send(`set zone ${getInt('zoneN',1,10)} x ${getInt('zoneX1',0,10000)} ${getInt('zoneX2',0,10000)}`)));$('capZoneXBtn').addEventListener('click',()=>safeProgram(()=>send(`cap zone ${getInt('zoneN',1,10)} x`)));
$('setZoneZBtn').addEventListener('click',()=>safeProgram(()=>send(`set zone ${getInt('zoneN',1,10)} z ${getInt('zoneZ1',0,10000)} ${getInt('zoneZ2',0,10000)}`)));$('capZoneZBtn').addEventListener('click',()=>safeProgram(()=>send(`cap zone ${getInt('zoneN',1,10)} z`)));
$('setZoneTimesBtn').addEventListener('click',()=>safeProgram(async()=>{const n=getInt('zoneN',1,10);await send(`zone ${n} dip ${getInt('zoneDip',0,36000)}`);await send(`zone ${n} wait ${getInt('zoneWait',0,36000)}`);await send(`zone ${n} tilt ${getInt('zoneTilt',0,5000)}`);}));
$('setZoneSpeedBtn').addEventListener('click',()=>safeProgram(()=>send(`zone ${getInt('zoneN',1,10)} speed ${getInt('zoneHPct',1,100)} ${getInt('zoneVPct',1,100)}`)));$('zoneOnBtn').addEventListener('click',()=>safeProgram(()=>send(`zone ${getInt('zoneN',1,10)} on`)));$('zoneOffBtn').addEventListener('click',()=>safeProgram(()=>send(`zone ${getInt('zoneN',1,10)} off`)));
$('zoneN').addEventListener('change',()=>{const z=state.zones.get(num($('zoneN').value));if(z)updateZone('@ZONE '+Object.entries(z).map(([k,v])=>`${k}=${v}`).join(' '));});
$('saveLogBtn').addEventListener('click',saveLog);$('clearLogBtn').addEventListener('click',()=>{logEl.textContent='';appendLog('Окно журнала очищено; сохранённый буфер не удалён','local');});$('sendManualBtn').addEventListener('click',()=>{const c=$('manualCommand').value.trim();if(c)send(c);});$('manualCommand').addEventListener('keydown',e=>{if(e.key==='Enter')$('sendManualBtn').click();});
if(!('serial'in navigator))$('browserWarning').classList.remove('hidden');
setConnected(false);updateGateUI();
