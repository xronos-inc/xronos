# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# pyright: standard

import asyncio
import json
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

# errors
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.templating import Jinja2Templates
from websockets.exceptions import ConnectionClosedError

try:
    from .montecarlo import PointQueue, SimulationRequest, run_sim
except ImportError:
    # The relative import does not work when running via uvicorn. Try again
    # using a regular import.
    from montecarlo import PointQueue, SimulationRequest, run_sim  # type: ignore


app = FastAPI()
templates = Jinja2Templates(directory="templates")
executor = ThreadPoolExecutor(max_workers=4)  # Adjust max_workers as needed.


@app.get("/", response_class=HTMLResponse)
async def get_visualization(request: Request):  # type: ignore
    return templates.TemplateResponse("index.html", {"request": request})


@app.get("/favicon.svg", include_in_schema=False)
@app.get("/favicon.ico", include_in_schema=False)
async def favicon() -> FileResponse:
    return FileResponse("static/images/favicon.svg")


async def run_simulation_in_thread(
    params: SimulationRequest, queue: PointQueue
) -> None:
    """Run the simulation in a separate thread.

    The goal here is to show how to build a live interface between xronos
    and the webpage.
    """
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(executor, run_sim, params, queue)


@app.websocket("/ws/{total_points}/{batch_size}/{batch_delay}/{tracing}")
async def websocket_endpoint(
    websocket: WebSocket,
    total_points: int,
    batch_size: int,
    batch_delay: int,
    tracing: bool,
) -> None:
    queue = PointQueue()
    await websocket.accept()

    sim_task: Optional[asyncio.Task] = None
    try:
        # Start the simulation in a separate thread
        sim_task = asyncio.create_task(
            run_simulation_in_thread(
                {
                    "total_points": total_points,
                    "batch_size": batch_size,
                    "batch_delay": batch_delay,
                    "enable_tracing": tracing,
                },
                queue=queue,
            )
        )

        # Start publishing points immediately
        points_sent = 0
        while points_sent < total_points:
            try:
                # check if the task is done, join if so.
                # This also has the advantages of raising any exceptions.
                if sim_task.done():
                    await sim_task
                point = queue.get_point()
                if point:
                    await websocket.send_text(json.dumps(point))
                    points_sent += 1
                else:
                    await asyncio.sleep(0.001)

            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f"Error during point processing: {e}")
                raise

    except WebSocketDisconnect:
        print("Client disconnected during connection setup")
    except asyncio.CancelledError:
        print("WebSocket connection cancelled")
    except ConnectionClosedError:
        print("Connection closed unexpectedly")
    except Exception as e:
        print(f"Unexpected error in websocket comms: {e}")
        raise
    finally:
        print("Closing websocket")
        await websocket.close()
        if sim_task is not None:
            sim_task.cancel()


@app.on_event("shutdown")
async def shutdown_event() -> None:
    executor.shutdown(wait=True)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000, timeout_graceful_shutdown=3)
