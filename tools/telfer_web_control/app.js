'use strict';
const $ = id => document.getElementById(id), names=['h1','h2','v1','v2'], labels=['X1','X2','Z1','Z2'];
const paramDefs=globalThis.HE200_PARAMETERS;
let port=null,reader=null,writer=null,reading=false,closing=false,rx='',supported=false,seq=0,epoch=0,txTail=Promise.resolve(),lastRx=0,lastHandshake=0;
let busy=false,localStop=false,jog=null,jogTimer=null,source='OFFLINE';
let controller={armed:'0',cal:'0',running:'0',paused:'0',pulse:'0',session:null},axes=Array(4).fill(null);
let draft=ProgramModel.demo(1),runtimeProgram=null,incomingProgram=null,formDirty=true,selectedZone=1,forceReload=false,forceKeepDraft=false,undoStack=[];
let simulation={},timeState={},memoryState={},slotNames=Array(4).fill(''),lastStateKey='',lastIssue='0';
const requests=new Map(),parameters=new Map(),logLines=[],events=[],trend=[];
const reasons={NOT_ARMED_OR_SAFETY:'Начните работу или устраните активный останов.',ARM_REQUIRES_STOPPED_FRESH_SAFETY:'Нужны остановленные ПЧ, четыре свежих координаты и неактивный E-STOP.',JOG_GATES:'Для движения нужны проверенные направления выбранных приводов.',JOG_REQUIRES_STOPPED:'Дождитесь полного останова.',JOG_FREQUENCY:'Скорость ниже P6.03 или P0.14. Прочитайте пусковые и частотные параметры.',JOG_PROFILE_MAX:'Скорость выше максимума профиля.',CALIBRATION_INPUT_OR_FREQUENCY:'Проверьте скорость, длительность импульса и P6.03/P0.14.',CALIBRATION_PREPARE_TIMEOUT:'Не получены стабильные измерения четырёх датчиков.',CALIBRATION_REJECTED:'Проверка направления отклонена.',PROGRAM_SPEED_BELOW_START_OR_LOWER_LIMIT:'Скорость программы или замедления ниже P6.03/P0.14.',VALUE_CHANGED_RELOAD:'Значение изменилось после чтения. Перечитайте параметр.',WRITE_VERIFY_FAILED:'Запись не подтверждена; управление отключено.',WRITE_REQUIRES_STOPPED_ALLOWED_RANGE:'Запись требует полного останова и допустимого диапазона.',SESSION:'Сеанс контроллера изменился. Подключитесь заново.',EDIT_REQUIRES_IDLE:'Дождитесь останова перед передачей изменений.',AUTO_GATES:'Нужны готовая программа, четыре направления, свежие координаты и останов.',INCOMPLETE_PROGRAM:'Не все параметры переданы или нет включённых зон.',PROGRAM_NAME:'Название слишком длинное или недопустимое.',SIM_BUILD_REQUIRED:'Доступно только в учебной сборке.',VFD_HEALTH:'Авария или потеря связи с ПЧ.',ESTOP_STILL_ACTIVE_OR_BUSY:'E-STOP ещё активен или система занята.'};
const phases={IDLE:'Готовность к запуску',PREP_TRAVEL:'Транспортная высота',MOVE_ZONE_H:'Переход к зоне',LOWER_TILT:'Погружение с наклоном',WAIT_AFTER_TILT:'Пауза после наклона',LOWER_BASE:'Погружение обеих сторон',WAIT_DIP:'Выдержка в зоне',RAISE_TILT_HIGH:'Подъём с наклоном',WAIT_DRAIN:'Стекание',RAISE_TRAVEL:'Подъём до транспортной высоты',NEXT_ZONE:'Следующая зона',DRY_GOTO_STAGING:'Промежуточная зона',DRY_WAIT_OPEN:'Откройте камеру',DRY_GOTO_DRY:'Переход к сушке',DRY_LOWER_DROP:'Опускание в сушку',DRY_WAIT_DETACH:'Отсоедините подвеску',DRY_RAISE_TRAVEL:'Подъём после отсоединения',DRY_WAIT_START:'Подтвердите начало сушки',DRY_WAIT_TIMER:'Сушка',DRY_LOWER_PICK:'Опускание к подвеске',DRY_WAIT_ATTACH:'Присоедините подвеску',DRY_RAISE_TRAVEL2:'Подъём из сушки',DRY_GOTO_STAGING2:'Возврат к промежуточной зоне',DRY_WAIT_CLOSE:'Закройте камеру',MOVE_HOME_Z:'HOME: транспортная высота',MOVE_HOME_X:'Возврат в HOME',DONE:'Программа завершена',FAULT:'Выполнение остановлено с ошибкой'};
const issues={'1':'Нет heartbeat от веб-пульта','2':'Потеряны данные датчика','3':'Активен E-STOP или его защёлка','4':'Авария или потеря связи с ПЧ','5':'Сработал концевик','6':'Ошибка проверки направления','7':'Не подтверждена запись параметра'};
function reason(code){return reasons[code]||code||'Команда отклонена';}
function kv(line){return Object.fromEntries(line.split(/\s+/).slice(1).map(t=>{const i=t.indexOf('=');return i>0?[t.slice(0,i),t.slice(i+1)]:null;}).filter(Boolean));}
function integer(value,min,max){const text=String(value);if(!/^-?\d+$/.test(text))throw Error('Требуется целое число');const n=Number(text);if(!Number.isSafeInteger(n)||n<min||n>max)throw Error(`Значение должно быть ${min}…${max}`);return n;}
function number(id,min,max){return integer($(id).value,min,max);}
function connected(){return !!writer&&supported;}
function isSim(){return source==='SIM';}
function armed(){return connected()&&!localStop&&controller.armed==='1'&&lastRx&&Date.now()-lastRx<2200;}
function moving(){return !!jog||controller.running==='1'||controller.pulse==='1'||Number(controller.jog)>0;}
function maskReady(mask){return (Number(controller.cal)&mask)===mask;}
function token(){if(!armed())throw Error('Сначала нажмите «Начать работу»');return controller.session;}
function note(text,bad=false){$('notice').textContent=text;$('notice').style.borderColor=bad?'var(--red)':'var(--blue)';}
function event(text,error=false){events.unshift({text,error,at:new Date().toLocaleTimeString('ru-RU',{hour12:false})});if(events.length>200)events.pop();renderEvents();}
function log(text,kind='RX'){logLines.push(`[${new Date().toLocaleTimeString('ru-RU',{hour12:false})}] ${kind} ${text}`);if(logLines.length>60000)logLines.splice(0,1000);if(!$('log').closest('details').open)return;$('log').textContent=logLines.slice(-1500).join('\n');if($('scrollLog').checked)$('log').scrollTop=$('log').scrollHeight;}
function cancelRequests(message){for(const r of requests.values()){clearTimeout(r.timer);r.reject(Error(message));}requests.clear();}
function resetSession(){epoch++;supported=false;localStop=true;cancelRequests('Команда отменена: подключение сброшено');stopJogTimer();jog=null;controller={armed:'0',cal:'0',running:'0',paused:'0',pulse:'0',session:null};axes=Array(4).fill(null);parameters.clear();$('external').checked=false;renderAxes();render();}
function write(cmd,priority=false){
 if(typeof cmd!=='string'||/[\r\n]/.test(cmd))return Promise.reject(Error('Недопустимая команда'));
 if(cmd!=='service info'&&!supported)return Promise.reject(Error('Нужна прошивка Step9L'));
 if(priority)epoch++;const ticket=epoch;
 const work=async()=>{if(ticket!==epoch)throw Error('Команда отменена');if(!writer)throw Error('COM отключён');log(cmd,'TX');await writer.write(new TextEncoder().encode(cmd+'\n'));};
 const promise=txTail.catch(()=>{}).then(work);txTail=promise;return promise;
}
function request(command,response='@WEB_ACK '){const id=++seq;return new Promise((resolve,reject)=>{const timer=setTimeout(()=>{requests.delete(id);reject(Error('Нет подтверждения команды'));},6000);requests.set(id,{resolve,reject,timer,response});write(`${command} @${id}`).catch(e=>{clearTimeout(timer);requests.delete(id);reject(e);});});}
function resolveReply(line,v){const r=requests.get(Number(v.id));if(!r||(!line.startsWith(r.response)&&!line.startsWith('@WEB_ACK ')))return;requests.delete(Number(v.id));clearTimeout(r.timer);if(v.ok==='0')r.reject(Error(reason(v.reason)||`Ошибка ${v.error}`));else r.resolve(v);}
async function action(fn){if(busy)return;busy=true;render();try{await fn();}catch(e){if(!e.message.startsWith('Команда отменена')){note(e.message,true);event(e.message,true);}}finally{busy=false;render();}}
async function stop(){localStop=true;stopJogTimer();jog=null;cancelRequests('Команда отменена: STOP');controller.armed='0';render();try{await write('web stop',true);event('Остановлено оператором');}catch(e){note(e.message,true);}}
function decodeName(hex){try{return new TextDecoder('utf-8',{fatal:true}).decode(Uint8Array.from(hex.match(/../g)||[],v=>parseInt(v,16)));}catch{return '';}}
function encodeName(name){return [...new TextEncoder().encode(name)].map(c=>c.toString(16).padStart(2,'0')).join('')||'-';}
function processLine(line){
 lastRx=Date.now();if(line.startsWith('FW:')){resetSession();if(writer)write('service info').catch(e=>note(e.message,true));return;}const v=kv(line);
 if(line.startsWith('@SERVICE_INFO ')){
  const version=v.fw==='v6-system-step9l-web-control',sim=v.build==='SIM'&&v.desktopSim==='1'&&v.physicalTx==='0'&&v.diagnosticLock==='1'&&v.auto==='0',field=v.build==='FIELD'&&v.desktopSim==='0'&&v.physicalTx==='1'&&v.diagnosticLock==='0'&&v.auto==='1';
  supported=version&&(sim||field)&&v.dwinMotion==='0';source=sim?'SIM':field?'FIELD':'UNKNOWN';$('firmware').textContent=`${v.fw||'—'} · ${v.build||'—'}`;
  if(supported){note(sim?'Учебная модель на Mega подключена. Начните работу; реальное оборудование не используется.':'Рабочая сборка подключена. Проверьте аппаратную защиту.');write('web info').catch(e=>note(e.message,true));}else note('Прошивка не поддерживает Step9L или не прошла проверку режима.',true);render();
 }else if(line.startsWith('@WEB_STATE ')){
  if(controller.session&&controller.session!==v.session){resetSession();if(writer)write('service info').catch(()=>{});}
  if(v.armed==='0'&&jog){jog.button?.classList.remove('held');jog=null;stopJogTimer();}controller=v;if(v.armed==='0')localStop=false;
  const key=`${v.running}:${v.paused}:${v.phase}:${v.zone}`;if(key!==lastStateKey){if(v.running==='1'||['DONE','FAULT'].includes(v.phase))event(`${phases[v.phase]||v.phase}${v.running==='1'?' · зона '+v.zone:''}${v.paused==='1'?' · пауза':''}`,v.phase==='FAULT');lastStateKey=key;}
  if(v.issue&&v.issue!=='0'&&v.issue!==lastIssue){const message=issues[v.issue]||'Останов системы';note(message,true);event(message,true);}lastIssue=v.issue||'0';render();
 }else if(line.startsWith('@WEB_AXIS ')){const i=Number(v.i);if(i<0||i>=4)return;axes[i]={...axes[i],...v};renderAxes();updateSettingsInputs(i,v);if(i===3)sampleTrend();}
 else if(line.startsWith('@HE200 ')){const i=names.indexOf((v.name||'').toLowerCase());if(i>=0){axes[i]={...axes[i],online:v.online,runHz:v.online==='1'&&Number.isFinite(Number(v.run001))?Number(v.run001)/100:undefined,setHz:v.online==='1'&&Number.isFinite(Number(v.set001))?Number(v.set001)/100:undefined,fault:v.fault};renderAxes();}}
 else if(line.startsWith('@WEB_PARAM ')){parameters.set(`${v.drive}:${v.key}`,v);renderParameter(v);resolveReply(line,v);}
 else if(line.startsWith('@WEB_ACK ')){resolveReply(line,v);if(v.ok==='0')note(reason(v.reason),true);}
 else if(line.startsWith('@WEB_WRITE '))event(`${v.drive.toUpperCase()} ${v.key}: ${v.before} → ${v.after}${isSim()?' (модель)':''}`,v.ok!=='1');
 else if(line.startsWith('@WEB_SLOT ')){slotNames[Number(v.slot)-1]=decodeName(v.nameHex||'');renderSlots();}
 else if(line.startsWith('@PROGRAM '))incomingProgram={...v,name:slotNames[Number(v.slot)-1]||`Программа ${v.slot}`,zones:[],order:[]};
 else if(line.startsWith('@ZONE ')&&incomingProgram)incomingProgram.zones[Number(v.n)-1]={...v,enabled:v.en,hPct:v.hpct,vPct:v.vpct};
 else if(line.startsWith('@WEB_ORDER ')&&incomingProgram)incomingProgram.order=(v.values||'').split(',').map(Number);
 else if(line.startsWith('@WEB_DRY ')&&incomingProgram)incomingProgram.drying=v;
 else if(line.startsWith('@WEB_PROGRAM_END')&&incomingProgram){try{runtimeProgram=ProgramModel.normalize(incomingProgram);controller.slot=String(runtimeProgram.slot);if(!forceKeepDraft&&(forceReload||!formDirty))fillEditor(runtimeProgram,false);forceReload=false;render();}catch(e){note('Не удалось прочитать программу: '+e.message,true);}incomingProgram=null;}
 else if(line.startsWith('@WEB_SIM ')){simulation=v;renderSimulation();}
 else if(line.startsWith('@WEB_TIME ')){timeState=v;render();}
 else if(line.startsWith('@WEB_MEMORY ')){memoryState=v;$('memory').textContent=`Свободно ${v.free} Б; минимум ${v.minimum} Б${isSim()?` · RS485 ${v.tx1==='0'?'отключён':'?'}, SPI ${v.spi==='0'?'отключён':'?'}`:''}`;}
 else if(line.startsWith('@CAL ')){$('calResult').textContent=`${v.drive} ${v.command}: ${v.deltaMm} мм; ${v.measured==='1'?'измерение принято':'не подтверждено'}${isSim()?' в модели':''}`;event($('calResult').textContent,v.measured!=='1');}
 else if(line.startsWith('@WEB_EVENT '))event(reason(v.code),v.level==='error');
}
async function readLoop(){const decoder=new TextDecoder();try{while(reading&&reader){const {value,done}=await reader.read();if(done)break;if(!value)continue;rx+=decoder.decode(value,{stream:true});let idx;while((idx=rx.indexOf('\n'))>=0){const line=rx.slice(0,idx).replace(/\r$/,'');rx=rx.slice(idx+1);log(line);processLine(line);}if(rx.length>131072){rx='';event('Длинная строка COM отброшена',true);}}}catch(e){note('Ошибка COM: '+e.message,true);event('Соединение с Mega потеряно',true);}finally{reading=false;if(port&&!closing)disconnect();}}
async function connect(){try{if(!navigator.serial)throw Error('Откройте пульт в Edge или Chrome через локальный сервер');resetSession();rx='';port=await navigator.serial.requestPort();await port.open({baudRate:115200,bufferSize:65536});writer=port.writable.getWriter();reader=port.readable.getReader();reading=true;readLoop();await write('service info');render();}catch(e){note(e.message,true);await disconnect();}}
async function disconnect(){if(closing)return;closing=true;stopJogTimer();jog=null;reading=false;try{if(writer&&supported)await write('web stop',true);}catch{}try{if(reader){await reader.cancel();reader.releaseLock();}}catch{}try{writer?.releaseLock();}catch{}try{await port?.close();}catch{}reader=writer=port=null;resetSession();closing=false;render();}
function selectTab(tab){if(jog)releaseJog();window.scrollTo(0,0);document.querySelectorAll('[data-panel]').forEach(p=>p.hidden=p.dataset.panel!==tab);document.querySelectorAll('[data-tab]').forEach(b=>b.classList.toggle('active',b.dataset.tab===tab));if(tab==='overview')renderMap();}
