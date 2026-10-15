"""POST /api/real2sim_home/room_map: room maps from the R2S Capture apps (Bearer long-lived token)."""
from __future__ import annotations

from http import HTTPStatus

from aiohttp import web

from homeassistant.components.http import HomeAssistantView

from .const import API_ROOM_MAP, DOMAIN


class RoomMapView(HomeAssistantView):
    url = API_ROOM_MAP
    name = "api:real2sim_home:room_map"
    requires_auth = True

    async def get(self, request: web.Request) -> web.Response:
        hub = request.app["hass"].data.get(DOMAIN, {}).get("hub")
        if hub is None:
            return self.json_message("Real2Sim Home is not set up", HTTPStatus.SERVICE_UNAVAILABLE)
        return self.json(hub.geometry())

    async def post(self, request: web.Request) -> web.Response:
        hub = request.app["hass"].data.get(DOMAIN, {}).get("hub")
        if hub is None:
            return self.json_message("Real2Sim Home is not set up", HTTPStatus.SERVICE_UNAVAILABLE)
        try:
            data = await request.json()
        except ValueError:
            return self.json_message("body must be JSON", HTTPStatus.BAD_REQUEST)
        try:
            room = await hub.async_add_room_map(data)
        except (ValueError, KeyError, TypeError, IndexError) as err:
            return self.json_message(f"invalid room map: {err}", HTTPStatus.BAD_REQUEST)
        return self.json({"id": room["id"], "name": room["name"], "area_id": room["area_id"],
                          "objects": len(room.get("objects", []))})
