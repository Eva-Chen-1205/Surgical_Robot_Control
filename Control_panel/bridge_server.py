"""
=== Robot Arm WebSocket Bridge Server ===
中繼伺服器：將 θ1~θ5 數據從控制端即時轉發給 Unity 數位雙生端

用法：
    pip install websockets
    python bridge_server.py

連線方式：
    控制端 (瀏覽器): ws://<IP>:9090/controller
    Unity 端:        ws://<IP>:9090/unity
"""

import asyncio
import websockets
import json
import socket
import time
import sys

# ============== Configuration ==============
PORT = 9090

# ============== Client Management ==============
unity_clients = set()
controller_client = None
last_theta_data = None  # Cache for late-joining Unity clients
message_count = 0
start_time = None


def get_local_ip():
    """Get the local IP address for LAN connections."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


async def handle_controller(websocket):
    """Handle the controller (browser) connection."""
    global controller_client, last_theta_data, message_count, start_time

    controller_client = websocket
    start_time = time.time()
    message_count = 0
    print(f"\n[CONTROLLER] Browser controller connected!")
    print(f"[CONTROLLER] Unity clients currently connected: {len(unity_clients)}")

    try:
        async for message in websocket:
            message_count += 1

            # Parse and validate the incoming theta data
            try:
                data = json.loads(message)
                last_theta_data = data

                # Log periodically (every 100 messages to avoid spam)
                if message_count % 100 == 0:
                    elapsed = time.time() - start_time
                    rate = message_count / elapsed if elapsed > 0 else 0
                    print(f"\r[BRIDGE] Messages relayed: {message_count} | "
                          f"Rate: {rate:.1f} msg/s | "
                          f"Unity clients: {len(unity_clients)} | "
                          f"θ1={data.get('theta1', 0):.4f}", end="", flush=True)

                # Broadcast to all connected Unity clients
                if unity_clients:
                    # Use gather for concurrent sending
                    await asyncio.gather(
                        *[client.send(message) for client in unity_clients.copy()],
                        return_exceptions=True
                    )

            except json.JSONDecodeError:
                # If not JSON, try forwarding raw message anyway
                last_theta_data = message
                if unity_clients:
                    await asyncio.gather(
                        *[client.send(message) for client in unity_clients.copy()],
                        return_exceptions=True
                    )

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        controller_client = None
        print(f"\n[CONTROLLER] Browser controller disconnected. "
              f"Total messages relayed: {message_count}")


async def handle_unity(websocket):
    """Handle a Unity client connection."""
    global last_theta_data

    unity_clients.add(websocket)
    client_addr = websocket.remote_address
    print(f"\n[UNITY] New Unity client connected from {client_addr[0]}:{client_addr[1]}")
    print(f"[UNITY] Total Unity clients: {len(unity_clients)}")

    # Send the last known theta data so Unity can initialize immediately
    if last_theta_data is not None:
        try:
            if isinstance(last_theta_data, dict):
                await websocket.send(json.dumps(last_theta_data))
            else:
                await websocket.send(str(last_theta_data))
            print(f"[UNITY] Sent cached theta data to new client")
        except Exception:
            pass

    try:
        # Keep connection alive; Unity might also send messages (e.g., heartbeat)
        async for message in websocket:
            # Unity can send heartbeat/status messages
            try:
                data = json.loads(message)
                if data.get("type") == "ping":
                    await websocket.send(json.dumps({
                        "type": "pong",
                        "controller_connected": controller_client is not None,
                        "unity_clients": len(unity_clients),
                        "total_messages": message_count
                    }))
            except (json.JSONDecodeError, Exception):
                pass

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        unity_clients.discard(websocket)
        print(f"\n[UNITY] Unity client {client_addr[0]}:{client_addr[1]} disconnected")
        print(f"[UNITY] Remaining Unity clients: {len(unity_clients)}")

import os

async def handle_save(websocket):
    """Handle saving files directly to local disk without browser pop-ups."""
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                filename = data.get("filename")
                content = data.get("content")
                if filename and content:
                    filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
                    with open(filepath, "w", encoding="utf-8") as f:
                        f.write(content)
                    print(f"[{filename}] successfully generated in folder.")
                    await websocket.send(json.dumps({"status": "success", "file": filename}))
                else:
                    await websocket.send(json.dumps({"status": "error", "error": "Missing filename or content"}))
            except Exception as e:
                print(f"[SAVE ERROR] Failed to save {filename if 'filename' in locals() else 'file'}: {str(e)}")
                await websocket.send(json.dumps({"status": "error", "error": str(e)}))
    except websockets.exceptions.ConnectionClosed:
        pass


async def handler(websocket, path=None):
    """Route connections based on URL path."""
    # websockets >= 11 passes path differently
    if path is None:
        path = websocket.path

    if path == "/controller":
        await handle_controller(websocket)
    elif path == "/unity":
        await handle_unity(websocket)
    elif path == "/save":
        await handle_save(websocket)
    else:
        # Default: show usage info and close
        await websocket.send(json.dumps({
            "error": "Invalid path. Use /controller or /unity",
            "usage": {
                "controller": f"ws://<IP>:{PORT}/controller",
                "unity": f"ws://<IP>:{PORT}/unity"
            }
        }))
        await websocket.close()


async def status_printer():
    """Periodically print server status."""
    while True:
        await asyncio.sleep(30)
        print(f"\n[STATUS] Controller: {'Connected' if controller_client else 'Disconnected'} | "
              f"Unity clients: {len(unity_clients)} | "
              f"Total messages: {message_count}")


async def main():
    local_ip = get_local_ip()

    print("=" * 60)
    print("   Robot Arm WebSocket Bridge Server")
    print("   機器人手臂 WebSocket 中繼伺服器")
    print("=" * 60)
    print()
    print(f"  Server running on port {PORT}")
    print(f"  Local IP: {local_ip}")
    print()
    print("  連線地址 (Connection URLs):")
    print(f"  ┌─────────────────────────────────────────────────┐")
    print(f"  │ Controller: ws://{local_ip}:{PORT}/controller  │")
    print(f"  │ Unity:      ws://{local_ip}:{PORT}/unity       │")
    print(f"  └─────────────────────────────────────────────────┘")
    print()
    print("  請將 Unity 端地址告訴同學！")
    print("  Tell your classmate the Unity URL above!")
    print()
    print("-" * 60)
    print("Waiting for connections...\n")

    # Start the WebSocket server on all interfaces (0.0.0.0) with unlimited payload size
    server = await websockets.serve(handler, "0.0.0.0", PORT, max_size=None, ping_interval=None)

    # Run status printer in background
    asyncio.create_task(status_printer())

    await server.wait_closed()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nServer stopped by user.")
        sys.exit(0)
