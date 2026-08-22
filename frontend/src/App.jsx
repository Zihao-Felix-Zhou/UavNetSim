import { useEffect, useRef, useState } from 'react'
import { Activity, ChevronDown, CirclePause, CirclePlay, Gauge, MapPinned, RadioTower, Square, Terminal, Trash2, Zap } from 'lucide-react'
import uavNetSimLogo from './assets/uavnetsim-logo.png'
import ScenePicker from './components/ScenePicker'
import SceneViewport from './components/SceneViewport'

const EMPTY_METRICS = {
  generated: 0, delivered: 0, pdr_percent: 0, e2e_delay_ms: 0,
  throughput_kbps: 0, average_hops: 0, collisions: 0, phy_success_percent: 0,
}
const ACTIVE_STATUSES = ['running', 'paused', 'starting', 'preparing', 'stopping']
const DEFAULT_LAYOUT = { left: 330, right: 320, log: 240 }

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value))
}

function sceneCoordinateBounds(scene) {
  const zValues = [0]
  scene.terrain?.vertices.forEach((point) => zValues.push(point.z))
  scene.features.forEach((feature) => {
    feature.footprint.forEach((point) => {
      zValues.push(point.z)
      if (feature.category === 'building') zValues.push(point.z + feature.height)
    })
  })
  return {
    x: [0, scene.size_x],
    y: [0, scene.size_y],
    z: [Math.min(...zValues), Math.max(...zValues)],
  }
}

async function responseError(response) {
  const payload = await response.json().catch(() => null)
  return new Error(payload?.detail || `Request failed with HTTP ${response.status}`)
}

function Field({ label, children }) {
  return <label className="field"><span>{label}</span>{children}</label>
}

function Metric({ label, value, unit, tone }) {
  return <div className="metric"><span>{label}</span><strong className={tone || ''}>{value}</strong><small>{unit}</small></div>
}

function LayerSection({ index, title, open, disabled, onToggle, children }) {
  return (
    <section className={`layer-section ${open ? 'open' : ''}`}>
      <button className="layer-heading" onClick={onToggle} aria-expanded={open}>
        <span><small>{index}</small>{title}</span>
        <ChevronDown size={15} />
      </button>
      {open && <fieldset className="layer-settings" disabled={disabled}>{children}</fieldset>}
    </section>
  )
}

function Toggle({ label, checked, disabled = false, onChange }) {
  return (
    <label className={`toggle-field ${disabled ? 'disabled' : ''}`}>
      <span>{label}</span>
      <input type="checkbox" checked={checked} disabled={disabled} onChange={(event) => onChange(event.target.checked)} />
      <i aria-hidden="true" />
    </label>
  )
}

function formatEvent(event) {
  const data = event.data
  if (event.event_type === 'packet_tx_started') return `PKT ${data.packet_id}  UAV ${data.source} -> ${data.destinations.join(', ')}`
  if (event.event_type === 'packet_rx_succeeded') return `RX ${data.packet_id}  ${data.sinr_db.toFixed(1)} dB`
  if (event.event_type === 'packet_rx_failed') return `DROP ${data.packet_id}  ${data.sinr_db.toFixed(1)} dB`
  if (event.event_type === 'channel_snapshot') {
    if (data.mode === 'offline') return `TRACE snapshot  ${data.trace_index}`
    if (data.mode === 'hybrid') return `HYBRID snapshot  ${data.los_link_count} LoS / ${data.nlos_link_count} NLoS  ${data.solve_time_ms.toFixed(0)} ms`
    return `RT snapshot  ${data.solve_time_ms.toFixed(0)} ms`
  }
  if (event.event_type === 'channel_trace_snapshot_ready') return `TRACE ${data.completed} / ${data.total}`
  if (event.event_type === 'channel_trace_ready') return data.cache_hit ? 'TRACE CACHE LOADED' : 'TRACE READY'
  if (event.event_type === 'packet_delivered') return `DELIVERED ${data.packet_id}  ${data.delay_ms.toFixed(1)} ms`
  if (event.event_type === 'runtime_status') return `STATUS  ${String(data.status || '').toUpperCase()}`
  if (event.event_type === 'simulation_failed') return data.error || 'SIMULATION FAILED'
  return event.event_type.replaceAll('_', ' ').toUpperCase()
}

function eventTone(event) {
  if (event.event_type.includes('failed') || event.event_type.includes('collision')) return 'failed'
  if (event.event_type.includes('succeeded') || event.event_type.includes('delivered') || event.event_type.endsWith('ready')) return 'succeeded'
  if (event.event_type.startsWith('channel_')) return 'channel'
  return ''
}

export default function App() {
  const [scene, setScene] = useState(null)
  const [options, setOptions] = useState({ routing: [], routing_parameters: {}, mac: [], mobility: [], traffic_pattern: [], channel_mode: [] })
  const [state, setState] = useState({ status: 'idle', nodes: [], metrics: EMPTY_METRICS, sim_time_us: 0, duration_us: 20e6 })
  const [events, setEvents] = useState([])
  const [arcs, setArcs] = useState([])
  const [selectedNode, setSelectedNode] = useState(null)
  const [scenePicker, setScenePicker] = useState(false)
  const [openLayers, setOpenLayers] = useState({ simulation: true, application: false, network: false, mac: false, physical: false })
  const [error, setError] = useState('')
  const [panelLayout, setPanelLayout] = useState(DEFAULT_LAYOUT)
  const [settings, setSettings] = useState({
    seed: 2025, node_count: 8, duration_seconds: 20, playback_speed: 1, uav_speed_mps: 10,
    uav_min_altitude_m: null, uav_max_altitude_m: null,
    initial_energy_j: 20000, traffic_pattern: 'Poisson', packet_arrival_rate: 5,
    routing: 'Greedy', routing_parameter_values: {}, mac: 'CSMA_CA', mobility: 'GaussMarkov3D',
    channel_mode: 'online',
    samples_per_source: 100000, sionna_max_depth: 4, sionna_frequency_samples: 32,
    sionna_los: true, sionna_specular_reflection: true, sionna_diffuse_reflection: false,
    sionna_refraction: false, sionna_diffraction: false, sionna_edge_diffraction: false,
    channel_snapshot_interval_ms: 100, channel_snapshot_displacement_m: 1,
  })
  const socketRef = useRef()
  const logStreamRef = useRef()
  const workspaceRef = useRef()
  const resizeCleanupRef = useRef()

  useEffect(() => {
    Promise.all([fetch('/api/scene').then((response) => response.json()), fetch('/api/options').then((response) => response.json())])
      .then(([loadedScene, loadedOptions]) => { setScene(loadedScene); setOptions(loadedOptions) })
      .catch((loadError) => setError(loadError.message))
    fetch('/api/simulation/state').then((response) => response.json()).then(setState)
  }, [])

  useEffect(() => {
    if (!scene) return
    const terrainPeak = Math.max(0, ...(scene.terrain?.vertices || []).map((point) => point.z))
    setSettings((current) => ({
      ...current,
      uav_min_altitude_m: 0,
      uav_max_altitude_m: Math.ceil(terrainPeak + 120),
    }))
  }, [scene])

  useEffect(() => () => resizeCleanupRef.current?.(), [])

  useEffect(() => {
    const stream = logStreamRef.current
    if (stream) stream.scrollTop = stream.scrollHeight
  }, [events, scene])

  useEffect(() => {
    let retry
    function connect() {
      const protocol = location.protocol === 'https:' ? 'wss' : 'ws'
      const socket = new WebSocket(`${protocol}://${location.host}/api/ws`)
      socketRef.current = socket
      socket.onmessage = (message) => {
        const payload = JSON.parse(message.data)
        setState((current) => ({ ...current, ...payload.state }))
        setEvents((current) => {
          const bySequence = new Map([...current, ...payload.events].map((event) => [event.sequence, event]))
          return [...bySequence.values()].sort((left, right) => left.sequence - right.sequence).slice(-80)
        })
        setArcs((current) => {
          let next = current.filter((arc) => Date.now() - arc.createdAt < 1600)
          payload.events.forEach((event) => {
            if (event.event_type === 'packet_tx_started') {
              event.data.destinations.forEach((destination) => next.push({
                id: event.data.transmission_id,
                source: event.data.source,
                destination,
                createdAt: Date.now(),
                status: 'active',
              }))
            }
            if (event.event_type === 'packet_rx_succeeded' || event.event_type === 'packet_rx_failed') {
              next = next.map((arc) => arc.id === event.data.transmission_id && arc.destination === event.data.destination
                ? { ...arc, status: event.event_type === 'packet_rx_succeeded' ? 'success' : 'failed' }
                : arc)
            }
          })
          return next.slice(-80)
        })
      }
      socket.onclose = () => { retry = window.setTimeout(connect, 800) }
    }
    connect()
    return () => { window.clearTimeout(retry); socketRef.current?.close() }
  }, [])

  async function command(path, body) {
    setError('')
    try {
      const response = await fetch(path, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: body ? JSON.stringify(body) : undefined,
      })
      if (!response.ok) throw await responseError(response)
      const nextState = await response.json()
      setState((current) => ({ ...current, ...nextState }))
      return true
    } catch (requestError) {
      setError(requestError.message)
      return false
    }
  }

  async function startSimulation() {
    if (
      !Number.isFinite(settings.uav_min_altitude_m)
      || !Number.isFinite(settings.uav_max_altitude_m)
      || settings.uav_min_altitude_m >= settings.uav_max_altitude_m
    ) {
      setError('Maximum UAV altitude must be greater than minimum UAV altitude')
      return
    }
    setEvents([])
    setArcs([])
    setSelectedNode(null)
    setState({
      status: 'starting',
      nodes: [],
      metrics: EMPTY_METRICS,
      sim_time_us: 0,
      duration_us: settings.duration_seconds * 1e6,
    })
    const definitions = options.routing_parameters[settings.routing] || {}
    const routingParameters = Object.fromEntries(Object.entries(definitions).map(([key, definition]) => [
      key,
      settings.routing_parameter_values[settings.routing]?.[key] ?? definition.default,
    ]))
    const runSettings = Object.fromEntries(
      Object.entries(settings).filter(([key]) => key !== 'routing_parameter_values'),
    )
    const started = await command('/api/simulation/start', { ...runSettings, routing_parameters: routingParameters })
    if (!started) {
      fetch('/api/simulation/state').then((response) => response.json()).then(setState)
    }
  }

  const metrics = { ...EMPTY_METRICS, ...(state.metrics || {}) }
  const selected = state.nodes.find((node) => node.id === selectedNode)
  const selectedSpeed = selected ? Math.hypot(...selected.velocity) : 0
  const simulationProgress = Math.min(100, (state.sim_time_us || 0) / (state.duration_us || settings.duration_seconds * 1e6) * 100)
  const traceProgress = state.preparation?.total ? state.preparation.completed / state.preparation.total * 100 : 0
  const progress = state.status === 'preparing' ? traceProgress : simulationProgress
  const statusLabel = state.status === 'starting' ? 'initializing RT' : state.status === 'preparing' ? 'building trace' : state.status
  const statusValue = state.status === 'preparing' ? `${traceProgress.toFixed(0)}%` : `${((state.sim_time_us || 0) / 1e6).toFixed(2)} s`
  const latestLink = events.findLast((event) => event.event_type.startsWith('packet_rx_'))
  const buildings = scene?.features.filter((feature) => feature.category === 'building').length || 0
  const terrainRelief = scene?.terrain ? Math.max(0, ...scene.terrain.vertices.map((point) => point.z)) : 0
  const coordinateBounds = scene ? sceneCoordinateBounds(scene) : null
  const settingsLocked = ACTIVE_STATUSES.includes(state.status)
  const routingParameterDefinitions = options.routing_parameters[settings.routing] || {}
  const setSetting = (key, value) => setSettings((current) => ({ ...current, [key]: value }))
  const toggleLayer = (layer) => setOpenLayers((current) => ({ ...current, [layer]: !current[layer] }))
  const setRoutingParameter = (key, value) => setSettings((current) => ({
    ...current,
    routing_parameter_values: {
      ...current.routing_parameter_values,
      [current.routing]: {
        ...current.routing_parameter_values[current.routing],
        [key]: value,
      },
    },
  }))

  function resizeLimits(kind, workspace, layout) {
    if (kind === 'left') return [280, Math.max(280, Math.min(480, workspace.width - layout.right - 360))]
    if (kind === 'right') return [280, Math.max(280, Math.min(460, workspace.width - layout.left - 360))]
    return [150, Math.max(150, Math.min(380, workspace.height - 300))]
  }

  function startResize(kind, pointerEvent) {
    if (window.matchMedia('(max-width: 980px)').matches) return
    pointerEvent.preventDefault()
    resizeCleanupRef.current?.()
    const workspace = workspaceRef.current.getBoundingClientRect()
    const start = { x: pointerEvent.clientX, y: pointerEvent.clientY, layout: panelLayout }
    const [minimum, maximum] = resizeLimits(kind, workspace, start.layout)
    const onMove = (event) => {
      const delta = kind === 'log' ? start.y - event.clientY : event.clientX - start.x
      const value = kind === 'right' ? start.layout.right - delta : start.layout[kind] + delta
      setPanelLayout((current) => ({ ...current, [kind]: clamp(value, minimum, maximum) }))
    }
    const cleanup = () => {
      window.removeEventListener('pointermove', onMove)
      window.removeEventListener('pointerup', cleanup)
      window.removeEventListener('pointercancel', cleanup)
      document.body.classList.remove('is-resizing', `is-resizing-${kind}`)
      resizeCleanupRef.current = null
    }
    resizeCleanupRef.current = cleanup
    document.body.classList.add('is-resizing', `is-resizing-${kind}`)
    window.addEventListener('pointermove', onMove)
    window.addEventListener('pointerup', cleanup)
    window.addEventListener('pointercancel', cleanup)
  }

  function resizeWithKeyboard(kind, keyEvent) {
    const directions = kind === 'log' ? { ArrowUp: 16, ArrowDown: -16 } : kind === 'right' ? { ArrowLeft: 16, ArrowRight: -16 } : { ArrowLeft: -16, ArrowRight: 16 }
    const delta = directions[keyEvent.key]
    if (!delta || !workspaceRef.current) return
    keyEvent.preventDefault()
    const workspace = workspaceRef.current.getBoundingClientRect()
    setPanelLayout((current) => {
      const [minimum, maximum] = resizeLimits(kind, workspace, current)
      return { ...current, [kind]: clamp(current[kind] + delta, minimum, maximum) }
    })
  }

  if (!scene) return <main className="loading"><RadioTower size={30} /><span>Loading scene</span>{error && <small>{error}</small>}</main>

  return (
    <main className="app-shell" style={{ '--left-panel-width': `${panelLayout.left}px`, '--right-panel-width': `${panelLayout.right}px`, '--log-panel-height': `${panelLayout.log}px` }}>
      <header className="topbar">
        <div className="brand"><img className="brand-logo" src={uavNetSimLogo} alt="UavNetSim" /></div>
        <div className="run-controls">
          <button className="primary-button" disabled={ACTIVE_STATUSES.includes(state.status)} onClick={startSimulation}><CirclePlay size={17} /> Run</button>
          <button className="icon-button" title={state.status === 'paused' ? 'Resume simulation' : 'Pause simulation'} disabled={!['running', 'paused'].includes(state.status)} onClick={() => command(state.status === 'paused' ? '/api/simulation/resume' : '/api/simulation/pause')}>
            {state.status === 'paused' ? <CirclePlay size={18} /> : <CirclePause size={18} />}
          </button>
          <button className="icon-button" title="Stop simulation" disabled={!['running', 'paused', 'starting', 'preparing'].includes(state.status)} onClick={() => command('/api/simulation/stop')}><Square size={16} /></button>
        </div>
        <div className="run-status"><i className={state.status} /><span>{statusLabel}</span><b>{statusValue}</b></div>
      </header>

      <section className="workspace" ref={workspaceRef}>
        <aside className="left-panel">
          <button className="scene-heading" onClick={() => setScenePicker(true)} title="Change scene">
            <span><small>ACTIVE SCENE</small><strong>{scene.name}</strong></span><MapPinned size={18} />
          </button>
          <div className="scene-facts" title="Local ENU coordinate bounds">
            {Object.entries(coordinateBounds).map(([axis, range]) => (
              <span key={axis}><b>{axis.toUpperCase()}</b>{range[0].toFixed(0)}-{range[1].toFixed(0)} m</span>
            ))}
            <small>{buildings} structures{scene.terrain ? ` / ${terrainRelief.toFixed(0)} m relief` : ''}</small>
          </div>
          <div className="layer-stack">
            <LayerSection index="01" title="SIMULATION / UAV" open={openLayers.simulation} disabled={settingsLocked} onToggle={() => toggleLayer('simulation')}>
              <div className="field-row"><Field label="Nodes"><input type="number" min="2" max="50" value={settings.node_count} onChange={(event) => setSetting('node_count', Number(event.target.value))} /></Field><Field label="Duration"><div className="input-unit"><input type="number" min="1" max="3600" value={settings.duration_seconds} onChange={(event) => setSetting('duration_seconds', Number(event.target.value))} /><span>s</span></div></Field></div>
              <Field label="Mobility model"><select value={settings.mobility} onChange={(event) => setSetting('mobility', event.target.value)}>{options.mobility.map((item) => <option key={item}>{item}</option>)}</select></Field>
              <div className="field-row"><Field label="UAV speed"><div className="input-unit"><input type="number" min="0.1" max="100" step="0.5" value={settings.uav_speed_mps} onChange={(event) => setSetting('uav_speed_mps', Number(event.target.value))} /><span>m/s</span></div></Field><Field label="Initial energy"><div className="input-unit"><input type="number" min="1" step="100" value={settings.initial_energy_j} onChange={(event) => setSetting('initial_energy_j', Number(event.target.value))} /><span>J</span></div></Field></div>
              <div className="field-row"><Field label="Min altitude (Z)"><div className="input-unit"><input type="number" min="0" max="10000" step="1" value={settings.uav_min_altitude_m ?? ''} onChange={(event) => setSetting('uav_min_altitude_m', Number(event.target.value))} /><span>m</span></div></Field><Field label="Max altitude (Z)"><div className="input-unit"><input type="number" min="0.1" max="10000" step="1" value={settings.uav_max_altitude_m ?? ''} onChange={(event) => setSetting('uav_max_altitude_m', Number(event.target.value))} /><span>m</span></div></Field></div>
              <div className="field-row"><Field label="Random seed"><input type="number" value={settings.seed} onChange={(event) => setSetting('seed', Number(event.target.value))} /></Field><Field label="Playback"><div className="segmented">{[0.5, 1, 2, 5].map((speed) => <button type="button" key={speed} className={settings.playback_speed === speed ? 'active' : ''} onClick={() => setSetting('playback_speed', speed)}>{speed}x</button>)}</div></Field></div>
            </LayerSection>

            <LayerSection index="02" title="APPLICATION LAYER" open={openLayers.application} disabled={settingsLocked} onToggle={() => toggleLayer('application')}>
              <Field label="Packet arrival"><div className="segmented two">{options.traffic_pattern.map((pattern) => <button type="button" key={pattern} className={settings.traffic_pattern === pattern ? 'active' : ''} onClick={() => setSetting('traffic_pattern', pattern)}>{pattern}</button>)}</div></Field>
              <Field label="Arrival rate per UAV"><div className="input-unit"><input type="number" min="0.01" max="1000" step="0.1" value={settings.packet_arrival_rate} onChange={(event) => setSetting('packet_arrival_rate', Number(event.target.value))} /><span>pkt/s</span></div></Field>
            </LayerSection>

            <LayerSection index="03" title="NETWORK LAYER" open={openLayers.network} disabled={settingsLocked} onToggle={() => toggleLayer('network')}>
              <Field label="Routing protocol"><select value={settings.routing} onChange={(event) => setSetting('routing', event.target.value)}>{options.routing.map((item) => <option key={item}>{item}</option>)}</select></Field>
              {Object.keys(routingParameterDefinitions).length > 0 && <div className="protocol-parameters">
                <span className="protocol-parameters__label">{settings.routing} PARAMETERS</span>
                <div className="field-row protocol-grid">
                  {Object.entries(routingParameterDefinitions).map(([key, definition]) => {
                    const value = settings.routing_parameter_values[settings.routing]?.[key] ?? definition.default
                    return <Field key={key} label={definition.label}><div className="input-unit"><input type="number" min={definition.minimum} max={definition.maximum} step={definition.step} value={value} onChange={(event) => setRoutingParameter(key, Number(event.target.value))} />{definition.unit && <span>{definition.unit}</span>}</div></Field>
                  })}
                </div>
              </div>}
            </LayerSection>

            <LayerSection index="04" title="MAC LAYER" open={openLayers.mac} disabled={settingsLocked} onToggle={() => toggleLayer('mac')}>
              <Field label="MAC protocol"><select value={settings.mac} onChange={(event) => setSetting('mac', event.target.value)}>{options.mac.map((item) => <option key={item}>{item}</option>)}</select></Field>
            </LayerSection>

            <LayerSection index="05" title="PHYSICAL LAYER" open={openLayers.physical} disabled={settingsLocked} onToggle={() => toggleLayer('physical')}>
              <Field label="Channel calculation"><div className="segmented three">{options.channel_mode.map((mode) => <button type="button" key={mode} className={settings.channel_mode === mode ? 'active' : ''} onClick={() => setSetting('channel_mode', mode)}>{mode}</button>)}</div></Field>
              <div className="field-row"><Field label="Max depth"><input type="number" min="0" max="32" value={settings.sionna_max_depth} onChange={(event) => setSetting('sionna_max_depth', Number(event.target.value))} /></Field><Field label="Samples / source"><input type="number" min="100" max="10000000" step="1000" value={settings.samples_per_source} onChange={(event) => setSetting('samples_per_source', Number(event.target.value))} /></Field></div>
              <div className="field-row"><Field label="Frequency samples"><input type="number" min="1" max="4096" value={settings.sionna_frequency_samples} onChange={(event) => setSetting('sionna_frequency_samples', Number(event.target.value))} /></Field><Field label="Snapshot interval"><div className="input-unit"><input type="number" min="0.1" max="60000" step="1" value={settings.channel_snapshot_interval_ms} onChange={(event) => setSetting('channel_snapshot_interval_ms', Number(event.target.value))} /><span>ms</span></div></Field></div>
              <Field label="Snapshot displacement"><div className="input-unit"><input type="number" min="0.01" max="1000" step="0.1" value={settings.channel_snapshot_displacement_m} onChange={(event) => setSetting('channel_snapshot_displacement_m', Number(event.target.value))} /><span>m</span></div></Field>
              <div className="toggle-grid">
                <Toggle label="Line of sight" checked={settings.channel_mode === 'hybrid' || settings.sionna_los} disabled={settings.channel_mode === 'hybrid'} onChange={(value) => setSetting('sionna_los', value)} />
                <Toggle label="Specular reflection" checked={settings.sionna_specular_reflection} onChange={(value) => setSetting('sionna_specular_reflection', value)} />
                <Toggle label="Diffuse reflection" checked={settings.sionna_diffuse_reflection} onChange={(value) => setSetting('sionna_diffuse_reflection', value)} />
                <Toggle label="Refraction" checked={settings.sionna_refraction} onChange={(value) => setSetting('sionna_refraction', value)} />
                <Toggle label="Diffraction" checked={settings.sionna_diffraction} onChange={(value) => setSetting('sionna_diffraction', value)} />
                <Toggle label="Edge diffraction" checked={settings.sionna_edge_diffraction} onChange={(value) => setSetting('sionna_edge_diffraction', value)} />
              </div>
            </LayerSection>
          </div>
        </aside>

        <section className="viewport" aria-label="3D simulation scene">
          <SceneViewport scene={scene} nodes={state.nodes} arcs={arcs} selectedNode={selectedNode} onSelectNode={setSelectedNode} />
          {selected && <div className="node-inspector"><button onClick={() => setSelectedNode(null)} title="Close">x</button><small>UAV {String(selected.id).padStart(2, '0')}</small><strong>{selected.energy_j.toFixed(0)} J</strong><span>X {selected.position[0].toFixed(1)} &nbsp; Y {selected.position[1].toFixed(1)} &nbsp; Z {selected.position[2].toFixed(1)}</span><span>Speed {selectedSpeed.toFixed(1)} m/s &nbsp; Queue {selected.queue_size}</span></div>}
        </section>

        <section className="log-panel" aria-label="Network runtime log">
          <div className="log-toolbar">
            <span><Terminal size={15} /> NETWORK LOG</span>
            <div><small>{events.length} EVENTS</small><button type="button" title="Clear network log" aria-label="Clear network log" onClick={() => setEvents([])}><Trash2 size={14} /></button></div>
          </div>
          <div className="log-stream" ref={logStreamRef}>
            {events.length === 0 && <div className="log-empty">Waiting for simulation events...</div>}
            {events.map((event) => <div key={event.sequence} className={`log-line ${eventTone(event)}`}><time>{(event.sim_time_us / 1e6).toFixed(3)}</time><code>{event.event_type}</code><span>{formatEvent(event)}</span></div>)}
          </div>
        </section>

        <aside className="right-panel">
          <div className="panel-heading"><span><Activity size={16} /> NETWORK</span><small>{state.status === 'preparing' ? 'PREP' : ACTIVE_STATUSES.includes(state.status) ? 'LIVE' : 'LATEST'}</small></div>
          <div className="metric-primary"><span>PDR</span><strong>{metrics.pdr_percent.toFixed(1)}<small>%</small></strong><div className="meter"><i style={{ width: `${metrics.pdr_percent}%` }} /></div></div>
          <div className="metric-grid">
            <Metric label="E2E delay" value={metrics.e2e_delay_ms.toFixed(1)} unit="ms" />
            <Metric label="Throughput" value={metrics.throughput_kbps.toFixed(0)} unit="kbps" />
            <Metric label="Delivered" value={metrics.delivered} unit={`/ ${metrics.generated}`} tone="success" />
            <Metric label="Collisions" value={metrics.collisions} unit="events" tone={metrics.collisions ? 'danger' : ''} />
          </div>
          <div className="link-readout">
            <div className="panel-heading"><span><Gauge size={16} /> LATEST LINK</span></div>
            {latestLink ? <><strong className={latestLink.event_type.endsWith('failed') ? 'danger' : 'success'}>{latestLink.data.sinr_db.toFixed(1)} dB</strong><span>UAV {latestLink.data.source}{' -> '}UAV {latestLink.data.destination}</span><small>CH {latestLink.data.channel} &nbsp; {latestLink.data.signal_dbm.toFixed(1)} dBm</small></> : <span className="muted">No reception yet</span>}
          </div>
          <div className="section-title static node-section-title"><span>NODES</span><b>{state.nodes.length}</b></div>
          <div className="node-list node-list--telemetry">
            {state.nodes.map((node) => <button key={node.id} className={selectedNode === node.id ? 'selected' : ''} onClick={() => setSelectedNode(node.id)}><i /><span>UAV {String(node.id).padStart(2, '0')}</span><small>{node.position[2].toFixed(0)} m</small></button>)}
          </div>
        </aside>

        <div className="resize-handle resize-handle--left" role="separator" aria-label="Resize configuration panel" aria-orientation="vertical" tabIndex="0" title="Drag to resize configuration panel" onPointerDown={(event) => startResize('left', event)} onKeyDown={(event) => resizeWithKeyboard('left', event)} onDoubleClick={() => setPanelLayout((current) => ({ ...current, left: DEFAULT_LAYOUT.left }))} />
        <div className="resize-handle resize-handle--right" role="separator" aria-label="Resize telemetry panel" aria-orientation="vertical" tabIndex="0" title="Drag to resize telemetry panel" onPointerDown={(event) => startResize('right', event)} onKeyDown={(event) => resizeWithKeyboard('right', event)} onDoubleClick={() => setPanelLayout((current) => ({ ...current, right: DEFAULT_LAYOUT.right }))} />
        <div className="resize-handle resize-handle--log" role="separator" aria-label="Resize network log" aria-orientation="horizontal" tabIndex="0" title="Drag to resize network log" onPointerDown={(event) => startResize('log', event)} onKeyDown={(event) => resizeWithKeyboard('log', event)} onDoubleClick={() => setPanelLayout((current) => ({ ...current, log: DEFAULT_LAYOUT.log }))} />
      </section>

      <footer className="timeline"><span>{state.status === 'preparing' ? 'TRACE' : `${((state.sim_time_us || 0) / 1e6).toFixed(2)} s`}</span><div><i style={{ width: `${progress}%` }} /></div><span>{state.status === 'preparing' ? `${state.preparation?.completed || 0}/${state.preparation?.total || 0}` : `${settings.duration_seconds.toFixed(0)} s`}</span><b><Zap size={14} /> PHY {metrics.phy_success_percent.toFixed(0)}%</b></footer>
      {error && <div className="toast" role="alert">{error}<button onClick={() => setError('')}>x</button></div>}
      {scenePicker && <ScenePicker scene={scene} onClose={() => setScenePicker(false)} onScene={setScene} />}
    </main>
  )
}
