"""Config + options flow for Real2Sim Home."""
from __future__ import annotations

from typing import Any

import voluptuous as vol

from homeassistant.config_entries import ConfigEntry, ConfigFlow, ConfigFlowResult, OptionsFlow
from homeassistant.core import callback
from homeassistant.helpers import selector

from .const import (
    CONF_FORWARD, CONF_MQTT_TOPIC, CONF_NIGHT_END, CONF_NIGHT_START, CONF_PERSONS, CONF_UDP_PORT, DEFAULT_MQTT_TOPIC,
    DEFAULT_UDP_PORT, DOMAIN, NAME, PERSON_FIELDS,
)


def _base_schema(d: dict[str, Any]) -> vol.Schema:
    return vol.Schema({
        vol.Optional(CONF_UDP_PORT, default=d.get(CONF_UDP_PORT, DEFAULT_UDP_PORT)): vol.All(vol.Coerce(int), vol.Range(min=0, max=65535)),
        vol.Optional(CONF_FORWARD, description={"suggested_value": d.get(CONF_FORWARD, "")}): str,
        vol.Optional(CONF_MQTT_TOPIC, default=d.get(CONF_MQTT_TOPIC, DEFAULT_MQTT_TOPIC)): str,
        vol.Optional(CONF_NIGHT_START, default=d.get(CONF_NIGHT_START, 22)): vol.All(vol.Coerce(int), vol.Range(min=0, max=23)),
        vol.Optional(CONF_NIGHT_END, default=d.get(CONF_NIGHT_END, 7)): vol.All(vol.Coerce(int), vol.Range(min=0, max=23)),
    })


class Real2SimHomeConfigFlow(ConfigFlow, domain=DOMAIN):
    VERSION = 1

    async def async_step_user(self, user_input: dict[str, Any] | None = None) -> ConfigFlowResult:
        await self.async_set_unique_id(DOMAIN)
        self._abort_if_unique_id_configured()
        if user_input is not None:
            return self.async_create_entry(title=NAME, data=user_input)
        return self.async_show_form(step_id="user", data_schema=_base_schema({}))

    @staticmethod
    @callback
    def async_get_options_flow(entry: ConfigEntry) -> OptionsFlow:
        return Real2SimHomeOptionsFlow()


class Real2SimHomeOptionsFlow(OptionsFlow):
    """Step 1: listener / night hours. Step 2: per person, the entities that say where they are and their vitals
    (left empty = found automatically on the person's phone / watch devices)."""

    def __init__(self) -> None:
        self._base: dict[str, Any] = {}

    async def async_step_init(self, user_input: dict[str, Any] | None = None) -> ConfigFlowResult:
        cur = {**self.config_entry.data, **self.config_entry.options}
        if user_input is not None:
            self._base = user_input
            if self.hass.states.async_entity_ids("person"):
                return await self.async_step_persons()
            return self.async_create_entry(data={**user_input, CONF_PERSONS: cur.get(CONF_PERSONS, {})})
        return self.async_show_form(step_id="init", data_schema=_base_schema(cur))

    async def async_step_persons(self, user_input: dict[str, Any] | None = None) -> ConfigFlowResult:
        cur = (self.config_entry.options or {}).get(CONF_PERSONS, {})
        people = sorted(self.hass.states.async_entity_ids("person"))
        if user_input is not None:
            persons: dict[str, dict[str, str]] = {}
            for key, val in user_input.items():
                pid, _, fld = key.partition("|")
                if val:
                    persons.setdefault(pid, {})[fld] = val
            return self.async_create_entry(data={**self._base, CONF_PERSONS: persons})
        ent = selector.EntitySelector(selector.EntitySelectorConfig(domain="sensor"))
        schema: dict[Any, Any] = {}
        for pid in people:
            for fld in PERSON_FIELDS:
                key = f"{pid}|{fld}"
                schema[vol.Optional(key, description={"suggested_value": cur.get(pid, {}).get(fld)})] = ent
        return self.async_show_form(step_id="persons", data_schema=vol.Schema(schema),
                                    description_placeholders={"people": ", ".join(people)})
