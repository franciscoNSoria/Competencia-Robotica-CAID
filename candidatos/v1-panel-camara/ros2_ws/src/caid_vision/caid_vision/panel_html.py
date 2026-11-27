"""Página del panel de la cámara (HTML, CSS y JS en un solo texto)."""

PAGINA = r"""<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Panel cámara CAID</title>
<style>
:root{--bg:#0f1115;--card:#171a21;--line:#262b36;--tx:#e6e8ee;--mut:#8b93a3;--ac:#4cc2ff;--ok:#3ddc84;--bad:#ff6b6b;--warn:#ffc857}
@media (prefers-color-scheme:light){:root{--bg:#f3f4f7;--card:#fff;--line:#dcdfe6;--tx:#1b1e25;--mut:#5d6575;--ac:#0b7bd1;--ok:#13a05a;--bad:#d63b3b;--warn:#b7791f}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.4 system-ui,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:10px 16px;border-bottom:1px solid var(--line)}
header h1{font-size:16px;margin:0;font-weight:600}.dot{width:10px;height:10px;border-radius:50%;background:var(--bad)}.dot.on{background:var(--ok)}
main{display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:12px;padding:12px 16px}
@media (max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
.card h2{margin:0 0 8px;font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--mut);font-weight:600}
#video{width:100%;display:block;border-radius:8px;background:#000;aspect-ratio:4/3;object-fit:contain}
.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:10px}
.stat{background:var(--bg);border-radius:8px;padding:8px 10px}.stat b{display:block;font-size:20px;font-variant-numeric:tabular-nums}.stat span{color:var(--mut);font-size:11px}
.side{display:flex;flex-direction:column;gap:12px}
.row{display:flex;align-items:center;justify-content:space-between;gap:8px;margin:6px 0}
.row label{color:var(--tx)}.row output{color:var(--mut);min-width:28px;text-align:right;font-variant-numeric:tabular-nums}
input[type=range]{flex:1;accent-color:var(--ac)}select{background:var(--bg);color:var(--tx);border:1px solid var(--line);border-radius:6px;padding:4px 6px}
.sw{position:relative;width:38px;height:22px;flex:none}.sw input{opacity:0;position:absolute;inset:0;margin:0;cursor:pointer;z-index:1}
.sw i{position:absolute;inset:0;background:var(--line);border-radius:12px;transition:.15s}.sw i:before{content:"";position:absolute;left:3px;top:3px;width:16px;height:16px;background:#fff;border-radius:50%;transition:.15s}
.sw input:checked+i{background:var(--ac)}.sw input:checked+i:before{transform:translateX(16px)}
button{width:100%;padding:10px;border:0;border-radius:8px;background:var(--ac);color:#fff;font-weight:600;cursor:pointer;font-size:14px}
button:hover{filter:brightness(1.1)}#msg{margin-top:8px;font-size:12px;color:var(--mut);min-height:16px}
.chips{display:flex;flex-wrap:wrap;gap:6px}.chip{padding:3px 9px;border-radius:14px;font-size:12px;border:1px solid var(--line)}
.chip.rojo{border-color:#ff5c5c;color:#ff7b7b}.chip.verde{border-color:#3ddc84;color:#3ddc84}.chip.azul{border-color:#4aa3ff;color:#4aa3ff}
#gauge{display:flex;gap:14px;align-items:center}#gauge svg{flex:none}.big{font-size:26px;font-weight:700;font-variant-numeric:tabular-nums}
.bars{display:flex;gap:4px;align-items:flex-end;height:34px;margin-top:6px}.bars div{flex:1;background:var(--ac);opacity:.75;border-radius:3px 3px 0 0;min-height:2px}
.mut{color:var(--mut)}
</style></head><body>
<header><span class="dot" id="dot"></span><h1>Panel cámara CAID</h1><span class="mut" id="conn">conectando…</span></header>
<main>
 <section>
  <div class="card"><img id="video" src="/video" alt="video en vivo">
   <div class="stats">
    <div class="stat"><b id="fpsRx">–</b><span>fps recibidos</span></div>
    <div class="stat"><b id="fpsPr">–</b><span>fps procesados</span></div>
    <div class="stat"><b id="ms">–</b><span>ms por cuadro</span></div>
    <div class="stat"><b id="rssi">–</b><span>señal (dBm)</span></div>
   </div>
   <div class="row" style="margin-top:10px"><button id="foto" style="width:auto;padding:8px 16px">Guardar foto</button><span id="fotoMsg" class="mut"></span></div></div>
 </section>
 <aside class="side">
  <div class="card"><h2>Rumbo</h2>
   <div id="gauge">
    <svg width="96" height="96" viewBox="-48 -48 96 96"><circle r="44" fill="none" stroke="var(--line)" stroke-width="2"/>
     <line x1="0" y1="-44" x2="0" y2="-38" stroke="var(--mut)" stroke-width="2"/>
     <g id="needle"><path d="M0,-36 L7,6 L0,0 L-7,6 Z" fill="var(--ac)"/></g></svg>
    <div><div class="big" id="rumbo">–</div><div class="mut" id="rumboTxt">sin dato</div></div>
   </div>
   <div class="bars" id="bars"></div>
   <div class="row mut"><span>izq <b id="lizq">–</b></span><span>centro <b id="lcen">–</b></span><span>der <b id="lder">–</b></span></div>
  </div>
  <div class="card"><h2>Detecciones</h2><div class="chips" id="chips"><span class="mut">nada</span></div></div>
  <div class="card"><h2>Capas</h2>
   <div class="row"><label>Cubos y canastos</label><span class="sw"><input type="checkbox" id="c_cubos" checked><i></i></span></div>
   <div class="row"><label>Flechas</label><span class="sw"><input type="checkbox" id="c_flechas" checked><i></i></span></div>
   <div class="row"><label>Ruta y rumbo</label><span class="sw"><input type="checkbox" id="c_ruta" checked><i></i></span></div>
   <div class="row"><label>Piso libre (verde)</label><span class="sw"><input type="checkbox" id="c_piso" checked><i></i></span></div>
  </div>
  <div class="card"><h2>Cámara</h2>
   <div class="row"><label>Espejar (izq ↔ der)</label><span class="sw"><input type="checkbox" id="hmirror"><i></i></span></div>
   <div class="row"><label>Voltear (arriba ↕ abajo)</label><span class="sw"><input type="checkbox" id="vflip"><i></i></span></div>
   <div class="row"><label>Rotar imagen <span class="mut">(solo panel)</span></label><select id="rot"><option value="0">0°</option><option value="90">90° ↻</option><option value="180">180°</option><option value="270">270° ↺</option></select></div>
   <div class="row"><label>Brillo</label><input type="range" id="brightness" min="-2" max="2"><output></output></div>
   <div class="row"><label>Contraste</label><input type="range" id="contrast" min="-2" max="2"><output></output></div>
   <div class="row"><label>Saturación</label><input type="range" id="saturation" min="-2" max="2"><output></output></div>
   <div class="row"><label>Calidad JPEG</label><input type="range" id="quality" min="4" max="40"><output></output></div>
   <div class="row"><label>Tope de fps</label><input type="range" id="fps_max" min="0" max="30"><output></output></div>
   <div class="row"><label>Resolución</label><select id="framesize">
     <option value="6">QVGA 320×240</option><option value="8">CIF 400×296</option><option value="9">HVGA 480×320</option><option value="10">VGA 640×480</option></select></div>
   <button id="guardar">Guardar configuración</button><div id="msg"></div>
  </div>
 </aside>
</main>
<script>
const $=id=>document.getElementById(id);
const post=(u,d)=>fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)}).then(r=>r.json());
const ajustes=['brightness','contrast','saturation','quality','fps_max'];
let tocando=null;
function salida(id){const o=$(id).parentNode.querySelector('output');if(o)o.textContent=$(id).value}
for(const id of ['hmirror','vflip']) $(id).onchange=e=>post('/api/camara',{var:id,val:e.target.checked?1:0});
for(const id of ajustes){const el=$(id);
  el.oninput=()=>{salida(id)};
  el.onpointerdown=()=>tocando=id;
  el.onchange=()=>{tocando=null;post('/api/camara',{var:id,val:+el.value})}}
$('rot').onchange=e=>post('/api/rotacion',{grados:+e.target.value});
$('foto').onclick=()=>{$('fotoMsg').textContent='guardando…';post('/api/foto',{}).then(r=>{$('fotoMsg').textContent=r.ok?'✔ '+r.ruta:'Error: '+r.error}).catch(()=>$('fotoMsg').textContent='Error de conexión')};
$('framesize').onchange=e=>post('/api/camara',{var:'framesize',val:+e.target.value});
for(const k of ['cubos','flechas','ruta','piso']) $('c_'+k).onchange=e=>post('/api/capas',{[k]:e.target.checked});
$('guardar').onclick=()=>{$('msg').textContent='guardando…';
  post('/api/guardar',{}).then(r=>{$('msg').textContent=r.ok?'✔ Guardado. camara_node lo aplica al conectarse.':'Error: '+r.error}).catch(()=>$('msg').textContent='Error de conexión')};
function pintarDetecciones(d){const c=$('chips');c.innerHTML='';let n=0;
  const add=(t,col)=>{const s=document.createElement('span');s.className='chip '+(col||'');s.textContent=t;c.appendChild(s);n++};
  (d.cubos||[]).forEach(x=>add('cubo '+x.color,x.color));
  (d.flechas||[]).forEach(x=>add('flecha '+x.color+' → '+x.direccion,x.color));
  (d.canastos||[]).forEach(x=>add('canasto '+x.color,x.color));
  if(!n){c.innerHTML='<span class="mut">nada</span>'}}
let primera=true;
async function tick(){
 try{const s=await (await fetch('/api/estado')).json();
  $('dot').className='dot'+(s.conectado?' on':'');$('conn').textContent=s.conectado?'cámara conectada':('sin video'+(s.error?': '+s.error:''));
  $('fpsRx').textContent=s.fps_recibidos;$('fpsPr').textContent=s.fps_proceso;$('ms').textContent=s.ms_proceso;
  const e=s.esp||{};$('rssi').textContent=e.rssi_dbm??'–';
  if(e.hmirror!==undefined&&tocando===null){
    $('hmirror').checked=!!e.hmirror;$('vflip').checked=!!e.vflip;
    for(const id of ajustes){if(tocando!==id&&e[id]!==undefined){$(id).value=e[id];salida(id)}}
    if(e.framesize!==undefined&&document.activeElement!==$('framesize'))$('framesize').value=e.framesize}
  if(document.activeElement!==$('rot')&&s.rotacion!==undefined)$('rot').value=s.rotacion;
  pintarDetecciones(s.detecciones||{});
  const r=s.ruta||{};
  if(r.rumbo_deg!=null){$('rumbo').textContent=(r.rumbo_deg>=0?'+':'')+r.rumbo_deg.toFixed(0)+'°';
    $('needle').setAttribute('transform','rotate('+Math.max(-80,Math.min(80,r.rumbo_deg))+')');
    $('rumboTxt').textContent=Math.abs(r.rumbo_deg)<5?'derecho':(r.rumbo_deg>0?'girar a la derecha':'girar a la izquierda')+' · confianza '+Math.round((r.confianza||0)*100)+'%'}
  else{$('rumbo').textContent='–';$('needle').setAttribute('transform','rotate(0)');$('rumboTxt').textContent=r.estado==='bloqueado'?'bloqueado: sin paso adelante':'sin dato'}
  const l=r.libre||{};for(const [k,i] of [['izq','lizq'],['centro','lcen'],['der','lder']])$(i).textContent=l[k]!=null?Math.round(l[k]*100)+'%':'–';
  const bars=$('bars');const pr=r.profundidad||[];
  if(bars.children.length!==pr.length){bars.innerHTML=pr.map(()=>'<div></div>').join('')}
  pr.forEach((v,i)=>{bars.children[i].style.height=Math.max(2,Math.round(v*34))+'px'});
  const cap=s.capas||{};
  for(const k of ['cubos','flechas','ruta','piso'])if(document.activeElement!==$('c_'+k))$('c_'+k).checked=!!cap[k];
 }catch(err){$('conn').textContent='panel sin respuesta'}
}
setInterval(tick,500);tick();
</script></body></html>
"""
