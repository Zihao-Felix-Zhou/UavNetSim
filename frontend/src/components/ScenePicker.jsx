import { useEffect, useMemo, useRef, useState } from 'react'
import { MapContainer, Rectangle, TileLayer, useMap, useMapEvents } from 'react-leaflet'
import { Crosshair, FileUp, LoaderCircle, Map, X } from 'lucide-react'

async function responseError(response) {
  const payload = await response.json().catch(() => null)
  return new Error(payload?.detail || `Request failed with HTTP ${response.status}`)
}

function BoundsTool({ active, bounds, onBounds, onComplete }) {
  const map = useMap()
  const start = useRef(null)

  useEffect(() => {
    if (!active) {
      start.current = null
      map.dragging.enable()
    }
    return () => map.dragging.enable()
  }, [active, map])

  useMapEvents({
    mousedown(event) {
      if (!active) return
      start.current = event.latlng
      map.dragging.disable()
      onBounds({ south: event.latlng.lat, north: event.latlng.lat, west: event.latlng.lng, east: event.latlng.lng })
    },
    mousemove(event) {
      if (!active || !start.current) return
      onBounds({
        south: Math.min(start.current.lat, event.latlng.lat),
        north: Math.max(start.current.lat, event.latlng.lat),
        west: Math.min(start.current.lng, event.latlng.lng),
        east: Math.max(start.current.lng, event.latlng.lng),
      })
    },
    mouseup(event) {
      if (!active || !start.current) return
      const nextBounds = {
        south: Math.min(start.current.lat, event.latlng.lat),
        north: Math.max(start.current.lat, event.latlng.lat),
        west: Math.min(start.current.lng, event.latlng.lng),
        east: Math.max(start.current.lng, event.latlng.lng),
      }
      start.current = null
      map.dragging.enable()
      if (nextBounds.north > nextBounds.south && nextBounds.east > nextBounds.west) {
        onBounds(nextBounds)
        onComplete()
      }
    },
  })
  const rectangle = bounds.north > bounds.south && bounds.east > bounds.west
    ? [[bounds.south, bounds.west], [bounds.north, bounds.east]]
    : null
  return rectangle ? <Rectangle bounds={rectangle} pathOptions={{ color: '#f3a63a', weight: active ? 2.5 : 2, fillOpacity: active ? 0.18 : 0.12 }} /> : null
}

export default function ScenePicker({ scene, onClose, onScene }) {
  const anchor = scene.anchor || { latitude: 23.1065, longitude: 113.3248 }
  const initialBounds = useMemo(() => scene.bounds || ({
    south: anchor.latitude - 0.002,
    north: anchor.latitude + 0.002,
    west: anchor.longitude - 0.002,
    east: anchor.longitude + 0.002,
  }), [anchor, scene.bounds])
  const [bounds, setBounds] = useState(initialBounds)
  const [selecting, setSelecting] = useState(false)
  const [busy, setBusy] = useState(false)
  const [build, setBuild] = useState(null)
  const [error, setError] = useState('')
  const file = useRef()

  async function submitOsm() {
    setBusy(true)
    setBuild({ progress: 0, stage: 'Submitting selected area' })
    setError('')
    try {
      const response = await fetch('/api/scene/osm', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name: 'OpenStreetMap Scene', bounds }),
      })
      if (!response.ok) throw await responseError(response)
      let currentBuild = await response.json()
      setBuild(currentBuild)
      while (currentBuild.status === 'queued' || currentBuild.status === 'running') {
        await new Promise((resolve) => setTimeout(resolve, 300))
        const statusResponse = await fetch(`/api/scene/osm/${currentBuild.id}`)
        if (!statusResponse.ok) throw await responseError(statusResponse)
        currentBuild = await statusResponse.json()
        setBuild(currentBuild)
      }
      if (currentBuild.status === 'failed') throw new Error(currentBuild.error || 'Scene build failed')
      onScene(currentBuild.scene)
      onClose()
    } catch (requestError) {
      setError(requestError.message)
      setBuild(null)
    } finally {
      setBusy(false)
    }
  }

  async function importFile(event) {
    const selected = event.target.files?.[0]
    if (!selected) return
    setBusy(true)
    setError('')
    try {
      const payload = JSON.parse(await selected.text())
      const response = await fetch('/api/scene/import', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      })
      if (!response.ok) throw await responseError(response)
      onScene(await response.json())
      onClose()
    } catch (requestError) {
      setError(requestError.message)
    } finally {
      setBusy(false)
      event.target.value = ''
    }
  }

  return (
    <div className="scene-dialog" role="dialog" aria-modal="true">
      <div className="scene-dialog__bar">
        <div><span className="eyebrow">SCENE SOURCE</span><h2>Geospatial workspace</h2></div>
        <button className="icon-button" onClick={onClose} disabled={busy} title="Close scene workspace"><X size={18} /></button>
      </div>
      <div className="scene-dialog__map">
        <MapContainer center={[anchor.latitude, anchor.longitude]} zoom={16} zoomControl={false} preferCanvas className={selecting ? 'map-selecting' : ''}>
          <TileLayer attribution='&copy; OpenStreetMap contributors' url="https://tile.openstreetmap.org/{z}/{x}/{y}.png" />
          <BoundsTool active={selecting} bounds={bounds} onBounds={setBounds} onComplete={() => setSelecting(false)} />
        </MapContainer>
        <button
          className={`map-select-button ${selecting ? 'active' : ''}`}
          aria-pressed={selecting}
          disabled={busy}
          onClick={() => setSelecting((value) => !value)}
          title={selecting ? 'Cancel area selection' : 'Draw a rectangular OSM area'}
        >
          <Crosshair size={17} />
          {selecting ? 'Cancel selection' : 'Select area'}
        </button>
      </div>
      <div className="scene-dialog__footer">
        {build && (
          <div className="scene-build-status" aria-live="polite">
            <div><span>{build.stage}</span><strong>{build.progress}%</strong></div>
            <div
              className="scene-build-progress"
              role="progressbar"
              aria-label="Scene build progress"
              aria-valuemin="0"
              aria-valuemax="100"
              aria-valuenow={build.progress}
            >
              <i style={{ width: `${build.progress}%` }} />
            </div>
          </div>
        )}
        <div className="bounds-grid">
          {['south', 'west', 'north', 'east'].map((key) => (
            <label key={key}><span>{key.toUpperCase()}</span><input type="number" step="0.0001" value={bounds[key]} disabled={busy} onChange={(event) => setBounds({ ...bounds, [key]: Number(event.target.value) })} /></label>
          ))}
        </div>
        <div className="scene-actions">
          {error && <span className="error-text">{error}</span>}
          <input ref={file} type="file" accept="application/json,.json" hidden onChange={importFile} />
          <button className="secondary-button" onClick={() => file.current?.click()} disabled={busy}><FileUp size={16} /> Import JSON</button>
          <button className="primary-button" onClick={submitOsm} disabled={busy}>
            {busy ? <LoaderCircle className="spin" size={16} /> : <Map size={16} />}
            {busy ? `Building ${build?.progress || 0}%` : 'Build from OSM'}
          </button>
        </div>
      </div>
    </div>
  )
}
