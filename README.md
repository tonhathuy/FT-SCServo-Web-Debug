# FT-SCServo-Web-Debug

🤖 Web-based debug & control dashboard for Feetech SCS/STS servos.

Inspired by [FT_SCServo_Debug_Qt](https://github.com/Kotakku/FT_SCServo_Debug_Qt)

## Features

- 🔍 **Servo Scanner** - Auto-detect all connected servos
- 🎮 **Position Control** - Intuitive slider + quick presets  
- 📈 **Real-time Monitoring** - Position, load, voltage, temperature
- ⚙️ **Calibration** - Set min/max/center positions
- 🔧 **Change Servo ID** - Modify servo ID (EEPROM)
- 💪 **Torque Control** - Enable/disable servo torque
- 🔄 **Range Test** - Automatic full range motion test

## Quick Start

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
# or: venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt

# Run the server
python main.py
```

Then open http://localhost:8081 in your browser.

## Hardware

- Feetech SCS/STS series servos (e.g., STS3215)
- USB to TTL adapter (e.g., Waveshare Bus Servo Adapter)

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/connect` | POST | Connect to serial port |
| `/api/disconnect` | POST | Disconnect |
| `/api/scan` | POST | Scan for servos |
| `/api/servos` | GET | List all servos |
| `/api/servo/{id}/status` | GET | Get servo status |
| `/api/servo/position` | POST | Set position |
| `/api/servo/torque` | POST | Set torque |
| `/api/servo/change-id` | POST | Change servo ID |
| `/ws` | WebSocket | Real-time updates |

## TODO

- [ ] EEPROM parameter editing (Amax, Vmax, Vmin, etc.)
- [ ] Servo programming mode
- [ ] Save/Load configurations
- [ ] Multi-servo group control
- [ ] Trajectory recording & playback

## License

MIT
