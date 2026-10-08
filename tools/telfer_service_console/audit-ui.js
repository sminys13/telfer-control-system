'use strict';

const audit = { scan:null, active:false, pending:false, rows:new Map(), addresses:[], received:0, complete:false, integrityErrors:0 };
const inputFunctions = {0:'Нет функции',1:'FWD',2:'REV',3:'Трёхпроводное управление',4:'JOG FWD',5:'JOG REV',8:'Выбег',9:'Сброс ошибки',10:'Пауза RUN',11:'Внешняя ошибка NO',18:'Смена источника частоты',20:'Смена источника команды',21:'Запрет разгона/торможения',32:'DC торможение',33:'Внешняя ошибка NC',36:'Внешний STOP 1',37:'Смена источника команды 2',46:'Скорость/момент',47:'Аварийный останов',48:'Внешний STOP 2',49:'DC торможение с замедлением'};

function auditValue(key,drive){
  const row=audit.rows.get(key)?.[drive];
  return row?.ok?row.value:null;
}
function parameterFrequencyHz(key,drive){
  const value=auditValue(key,drive),resolution=auditValue('P0.22',drive);
  if(value===null||![1,2].includes(resolution))return null;
  return value*(resolution===1?0.1:0.01);
}

function safeConsoleCommand(cmd){
  // This UI is a diagnostic release, including when connected to OLD firmware.
  if (typeof cmd!=='string' || /[\r\n]/.test(cmd)) return false;
  if (/^(service info|service gate status|prog show|laser status|report|log (quiet|normal|verbose)|test (all|h1|h2|v1|v2))$/.test(cmd)) return true;
  return auditSupported && /^he200 audit (all|cancel)$/.test(cmd);
}
function resetDiagnosticSession(){
  diagnosticLock=true; auditSupported=false; sourceConfirmed=false; pulsePending=false;
  audit.scan=null; audit.active=false; audit.pending=false; audit.rows.clear(); audit.addresses=[];
  audit.received=0; audit.complete=false; audit.integrityErrors=0;
  state.gates={preflight:false,preflightResult:'wait',preflightReason:'',protocol:0,pulsePos:0,pulseNeg:0,confirmPos:0,confirmNeg:0,pair:0,assist:'OFF'};
  state.protocol={H1:null,H2:null,V1:null,V2:null};
  $('sourceConfirm').checked=false;
  $('auditStatus').textContent='Снимка нет. Требуется новый опрос четырёх ПЧ.';
  $('auditFindings').textContent='Нет данных текущего подключения.';
  renderAudit(); updateGateUI();
}
function updateAuditControls(){
  $('auditAllBtn').disabled=!port || !auditSupported || audit.active || audit.pending;
  $('auditCancelBtn').disabled=!port || !auditSupported || !(audit.active || audit.pending);
}
function formatAuditValue(meta,row){
  if (!row) return '—';
  if (!row.ok) return `НЕИЗВЕСТНО · err=${row.error} ex=${row.exception}`;
  const raw=`${row.value} / 0x${row.value.toString(16).toUpperCase().padStart(4,'0')}`;
  if(['SET_HZ','RUN_HZ','FREQ_A','FREQ_B'].includes(meta.name))
    return `${raw} · ${(row.value/100).toFixed(2)} Гц${meta.name==='SET_HZ'?' (задание)':meta.name==='RUN_HZ'?' (выход)':''}`;
  const maps={
    'P0.02':{0:'Панель',1:'Клеммы',2:'Связь'},
    'P0.03':{9:'Связь'}, 'P0.27':{0:'Нет привязки'},
    'P4.11':{0:'Двухпроводный 1',1:'Двухпроводный 2',2:'Трёхпроводный 1',3:'Трёхпроводный 2'},
    'P6.00':{0:'Прямой пуск',1:'Поиск скорости',2:'Предвозбуждение'},
    'P8.13':{0:'Реверс разрешён',1:'Реверс запрещён'},
    'P8.14':{0:'Нижний предел',1:'Останов',2:'Нулевая скорость'},
    'A0.00':{0:'Скорость',1:'Момент'},'STATUS':{1:'FWD',2:'REV',3:'STOP'}
  };
  const map=/^P4\.0[0-5]$/.test(meta.name)||/^A1\.0[0-4]$/.test(meta.name) ? inputFunctions : maps[meta.name];
  const label=map?.[row.value];
  // No inferred DI bit mapping, no inferred brake or safety status.
  return label ? `${raw} · ${label}` : raw;
}
function renderAudit(){
  for(let i=0;i<4;i++) document.querySelector('#auditTable thead tr').children[i+1].textContent=
    audit.addresses[i] ? `${driveNames[i]} · ${audit.addresses[i]}` : driveNames[i];
  const body=document.querySelector('#auditTable tbody');body.replaceChildren();
  for(const meta of HE200_AUDIT_MAP){
    const tr=document.createElement('tr');tr.dataset.key=meta.name;
    const title=document.createElement('td');title.textContent=`${meta.name} · 0x${meta.reg.toString(16).toUpperCase()} · ${meta.label}`;tr.appendChild(title);
    const values=audit.rows.get(meta.name)||[];
    for(let i=0;i<4;i++){
      const td=document.createElement('td');td.textContent=formatAuditValue(meta,values[i]);
      if(values[i]&&!values[i].ok)td.className='cell-bad';
      if(values[i]?.ok && meta.expected!==null && values[i].value!==meta.expected)td.className='cell-bad';
      tr.appendChild(td);
    }
    const comparison=document.createElement('td');
    const known=values.length===4 && [0,1,2,3].every(i=>values[i]?.ok);
    comparison.textContent=!known?'Неполные данные':values.every(v=>v.value===values[0].value)?'Одинаково':'РАЗЛИЧАЮТСЯ';
    if(known&&!values.every(v=>v.value===values[0].value))comparison.className='cell-bad';
    if(meta.expected!==null)comparison.textContent+=` · заявлено ${meta.expected}`;
    tr.appendChild(comparison);body.appendChild(tr);
  }
  updateAuditControls();
}
function auditFindings(){
  const lines=[];
  const get=auditValue;
  for(let i=0;i<4;i++){
    const notes=[];
    if(get('P0.02',i)!==null&&get('P0.02',i)!==2)notes.push('источник команд не «связь»');
    if(get('P0.03',i)!==null&&get('P0.03',i)!==9)notes.push('основная частота не «связь»');
    if(get('P4.02',i)===10)notes.push('X3 назначен «пауза RUN»; активность не доказана');
    if(get('P8.13',i)===1)notes.push('реверс запрещён (не объясняет отказ FWD)');
    if(get('A0.00',i)===1)notes.push('выбран режим момента');
    const startHz=parameterFrequencyHz('P6.03',i),setRaw=get('SET_HZ',i),setHz=setRaw===null?null:setRaw/100;
    const lowHz=parameterFrequencyHz('P0.14',i);
    if(startHz!==null&&setHz!==null&&setHz<startHz)
      notes.push(`задание ${setHz.toFixed(2)} Гц ниже стартовой частоты P6.03=${startHz.toFixed(2)} Гц — основной кандидат для проверки отказа пуска; это не подтверждение причины по результату RUN`);
    if(lowHz!==null&&setHz!==null&&setHz<lowHz&&get('P8.14',i)===0)
      notes.push(`задание ниже P0.14=${lowHz.toFixed(2)} Гц; P8.14=0 задаёт работу на нижнем пределе, поэтому «останов ниже предела» не подтверждён`);
    if(get('P6.00',i)===2){
      const current=get('P6.05',i),time=get('P6.06',i);
      notes.push(`предвозбуждение${current===null?'':': '+current+'%'}${time===null?'':', '+(time/10).toFixed(1)+' с'}`);
    }
    if(get('P0.09',i)===1)notes.push('ПЧ инвертирует направление (P0.09=1); знак определять отдельно по лазеру');
    if(get('P0.18',i)===0)notes.push('P0.18=0: время торможения 1 равно нулю; требуется учесть перед любым испытанием движения');
    if(get('STATUS',i)===3&&get('RUN_HZ',i)===0)
      notes.push('на момент чтения STOP, выходная частота 0 Гц; показание задания не означает RUN');
    const low=get('P0.14',i),mode=get('P8.14',i);
    if(low!==null&&low>0&&(mode===1||mode===2))notes.push('настроен STOP/нулевая скорость ниже нижнего предела; сопоставить P0.14, P0.22 и задание во время отказа. Текущий снимок не доказывает причину прежнего отказа');
    const fault=get('FAULT',i);if(fault!==null&&fault!==0)notes.push('есть код ошибки: '+fault);
    lines.push(`${driveNames[i]}: ${notes.join('; ')||'недостаточно данных для вывода о блокировке'}.`);
  }
  lines.push('Одинаковые параметры не доказывают исправность аппаратной цепи. DI/DO, реле, тормоза и независимый E-STOP требуют проверки на оборудовании. Разрешение движения: НЕТ.');
  $('auditFindings').textContent=lines.join('\n');
}
function processAudit(line){
  const v=parseKV(line);
  const invalid=()=>{
    ++audit.integrityErrors; audit.complete=false;
    $('auditStatus').textContent='Нарушена целостность снимка: несовместимый или противоречивый ответ. Сохраните журнал и повторите чтение.';
  };
  if(line.startsWith('@AUDIT_ROW ')){
    if(!audit.active||v.scan!==audit.scan)return;
    const meta=HE200_AUDIT_MAP.find(x=>x.name===v.key);
    const i=Number(v.drive);
    if(!meta||!Number.isInteger(i)||i<0||i>3||num(v.reg,-1)!==meta.reg){invalid();return;}
    const value=Number(v.value), ok=v.ok==='1'&&Number.isInteger(value)&&value>=0&&value<=65535;
    const row={ok,value:ok?value:null,error:num(v.error,-1),exception:num(v.exception,-1),addr:num(v.addr,-1),ms:num(v.ms,-1)};
    if(!Number.isInteger(row.addr)||row.addr<1||row.addr>247||
       !Number.isInteger(row.ms)||row.ms<0||!['0','1'].includes(v.ok)||
       (v.ok==='1'&&(!ok||row.error!==0||row.exception!==0))||
       (v.ok==='0'&&(!Number.isInteger(row.error)||row.error<=0))||
       (audit.addresses[i]!==undefined&&audit.addresses[i]!==row.addr)||
       audit.addresses.some((addr,index)=>index!==i&&addr===row.addr)){invalid();return;}
    audit.addresses[i]=row.addr;
    document.querySelector('#auditTable thead tr').children[i+1].textContent=`${driveNames[i]} · ${row.addr}`;
    const values=audit.rows.get(meta.name)||[];
    if(values[i]){
      if(JSON.stringify(values[i])!==JSON.stringify(row))invalid();
      return; // An identical retransmitted log line does not count twice.
    }
    ++audit.received;
    values[i]=row;audit.rows.set(meta.name,values);
    $('auditStatus').textContent=`Чтение: ${audit.received}/${HE200_AUDIT_MAP.length*4}. FC03, без записи.`;
    // One row at a time avoids rebuilding the full table for every serial frame.
    const tr=document.querySelector(`#auditTable tr[data-key="${meta.name}"]`);
    if(tr){const td=tr.children[i+1];td.textContent=formatAuditValue(meta,row);td.className=(!ok||(meta.expected!==null&&value!==meta.expected))?'cell-bad':'';}
    return;
  }
  if(v.state==='START'){
    if(!auditSupported||!/^\d+$/.test(v.scan||'')||num(v.rows,-1)!==HE200_AUDIT_MAP.length||
       v.drives!=='4'||v.fc!=='03'||v.writes!=='0'||v.motionPermit!=='0'){
      audit.active=false;audit.pending=false;invalid();updateAuditControls();return;
    }
    audit.scan=v.scan;audit.active=true;audit.pending=false;audit.complete=false;
    audit.rows.clear();audit.addresses=[];audit.received=0;audit.integrityErrors=0;renderAudit();
    $('auditStatus').textContent='Чтение всех четырёх приводов началось. Разрешение движения не выдаётся.';
    $('auditFindings').textContent='Сбор данных…';
  }else if(v.state==='DONE'&&v.scan===audit.scan&&audit.active){
    audit.active=false;audit.pending=false;
    const errors=[...audit.rows.values()].flat().filter(row=>row&&!row.ok).length;
    if(!/^\d+$/.test(v.errors||'')||Number(v.errors)!==errors)invalid();
    audit.complete=audit.received===HE200_AUDIT_MAP.length*4&&audit.integrityErrors===0;
    $('auditStatus').textContent=`Чтение завершено: ${audit.received}/${HE200_AUDIT_MAP.length*4}; ошибок чтения: ${v.errors}; нарушений целостности: ${audit.integrityErrors}. ${audit.complete?'Все ответы учтены.':'НЕПОЛНЫЙ ИЛИ НЕДОСТОВЕРНЫЙ СНИМОК.'} Сохраните журнал.`;
    renderAudit();auditFindings();
    if(audit.integrityErrors)$('auditFindings').textContent='Снимок недостоверен. Выводы о блокировке ПЧ не формируются.';
  }else if((v.state==='CANCELLED'&&v.scan===audit.scan)||
           (v.state==='REJECTED'&&audit.pending&&!audit.active)){
    audit.active=false;audit.pending=false;audit.complete=false;
    $('auditStatus').textContent=`Чтение ${v.state==='CANCELLED'?'прервано':'не запущено'}: ${v.reason||'по команде'}. Данные неполные.`;
    renderAudit();
  }
  updateAuditControls();
}
function showCalibration(line){
  const v=parseKV(line);
  $('calibrationResult').textContent=`${v.drive}: ${v.command}, изменение расстояния ${v.deltaMm} мм; измерение ${v.measured==='1'?'принято':'НЕ ПОДТВЕРЖДЕНО'}; калибровка ${v.ready==='1'?'двух направлений подтверждена':'не завершена'}; «+» → ${v.plusCommand}. Разрешение движения не выдаётся.`;
}
document.addEventListener('DOMContentLoaded',()=>{
  renderAudit();
  $('auditAllBtn').addEventListener('click',async()=>{
    if(!port||!auditSupported||audit.active||audit.pending)return;
    audit.pending=true;updateAuditControls();
    try{if(!await send('he200 audit all'))audit.pending=false;}
    catch(e){audit.pending=false;appendLog('Ошибка отправки: '+e.message,'local');}
    updateAuditControls();
  });
  $('auditCancelBtn').addEventListener('click',()=>send('he200 audit cancel'));
});
