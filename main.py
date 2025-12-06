"""
Servo Test Dashboard - FastAPI Backend
"""
import asyncio
import json
from typing import List, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
from pydantic import BaseModel
from servo_controller import controller, ServoInfo

app = FastAPI(title="Servo Test Dashboard")


class ConnectionRequest(BaseModel):
    port: str = '/dev/tty.usbmodem5A7C1167091'
    baudrate: int = 1000000


class PositionRequest(BaseModel):
    servo_id: int
    position: int
    time_ms: int = 500


class TorqueRequest(BaseModel):
    servo_id: int
    enable: bool


class CalibrationRequest(BaseModel):
    servo_id: int
    min_position: int
    max_position: int
    center_position: int


class ScanRequest(BaseModel):
    start_id: int = 1
    end_id: int = 20


class ChangeIdRequest(BaseModel):
    old_id: int
    new_id: int


# WebSocket connections for real-time updates
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except:
                pass


manager = ConnectionManager()


# API Endpoints
@app.get("/", response_class=HTMLResponse)
async def root():
    return FileResponse("static/index.html")


@app.post("/api/connect")
async def connect(req: ConnectionRequest):
    controller.port = req.port
    controller.baudrate = req.baudrate
    success, message = controller.connect()
    return {"success": success, "message": message}


@app.post("/api/disconnect")
async def disconnect():
    controller.disconnect()
    return {"success": True, "message": "Disconnected"}


@app.get("/api/status")
async def get_status():
    return {
        "connected": controller.is_connected,
        "port": controller.port,
        "baudrate": controller.baudrate,
        "servo_count": len(controller.servos)
    }


@app.post("/api/scan")
async def scan_servos(req: ScanRequest):
    if not controller.is_connected:
        raise HTTPException(status_code=400, detail="Not connected")
    
    servos = controller.scan_servos(req.start_id, req.end_id)
    return {"servos": [{"id": s.id, "model_number": s.model_number} for s in servos]}


@app.get("/api/servos")
async def get_servos():
    return {"servos": controller.get_all_servos()}


@app.get("/api/servo/{servo_id}/ping")
async def ping_servo(servo_id: int):
    success, message = controller.ping(servo_id)
    return {"success": success, "message": message}


@app.get("/api/servo/{servo_id}/status")
async def get_servo_status(servo_id: int):
    status = controller.read_status(servo_id)
    if status is None:
        raise HTTPException(status_code=404, detail="Servo not found")
    return status


@app.post("/api/servo/position")
async def set_position(req: PositionRequest):
    success, message = controller.set_position(req.servo_id, req.position, req.time_ms)
    await manager.broadcast({
        "type": "position_update",
        "servo_id": req.servo_id,
        "position": req.position
    })
    return {"success": success, "message": message}


@app.post("/api/servo/torque")
async def set_torque(req: TorqueRequest):
    success, message = controller.set_torque(req.servo_id, req.enable)
    return {"success": success, "message": message}


@app.post("/api/servo/{servo_id}/center")
async def center_servo(servo_id: int):
    success, message = controller.center_servo(servo_id)
    return {"success": success, "message": message}


@app.post("/api/servo/{servo_id}/test-range")
async def test_range(servo_id: int, min_pos: int = 0, max_pos: int = 4095):
    success, message = controller.test_range(servo_id, min_pos, max_pos)
    return {"success": success, "message": message}


@app.post("/api/servo/calibration")
async def set_calibration(req: CalibrationRequest):
    controller.set_calibration(
        req.servo_id,
        req.min_position,
        req.max_position,
        req.center_position
    )
    return {"success": True, "message": "Calibration saved"}


@app.post("/api/servo/change-id")
async def change_servo_id(req: ChangeIdRequest):
    """
    Change servo ID. 
    ⚠️ IMPORTANT: Only connect ONE servo at a time when changing ID!
    """
    success, message = controller.change_servo_id(req.old_id, req.new_id)
    if success:
        await manager.broadcast({
            "type": "id_changed",
            "old_id": req.old_id,
            "new_id": req.new_id
        })
    return {"success": success, "message": message}


# WebSocket for real-time updates
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            # Send status updates every 100ms
            if controller.is_connected and controller.servos:
                updates = []
                for servo_id in controller.servos:
                    status = controller.read_status(servo_id)
                    if status:
                        updates.append(status)
                
                if updates:
                    await websocket.send_json({
                        "type": "status_update",
                        "servos": updates
                    })
            
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        manager.disconnect(websocket)


# Mount static files
app.mount("/static", StaticFiles(directory="static"), name="static")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8081)

