"""Constants for Real2Sim Home."""
DOMAIN = "real2sim_home"
NAME = "Real2Sim Home"

CONF_UDP_PORT = "udp_port"
CONF_FORWARD = "forward"
CONF_NIGHT_START = "night_start"
CONF_NIGHT_END = "night_end"
CONF_PERSONS = "persons"
CONF_MQTT_TOPIC = "mqtt_topic"

DEFAULT_UDP_PORT = 5005
DEFAULT_MQTT_TOPIC = "real2sim/room_map"

STORAGE_KEY = "real2sim_home"
STORAGE_VERSION = 1

URL_BASE = "/real2sim_home_static"
CARD_FILE = "real2sim-home-card.js"
API_ROOM_MAP = "/api/real2sim_home/room_map"
PANEL_PATH = "home-map"

SIGNAL_UPDATE = "real2sim_home_update"
SIGNAL_NEW_ROOM = "real2sim_home_new_room"
SIGNAL_NEW_NODE = "real2sim_home_new_node"
SIGNAL_NEW_PERSON = "real2sim_home_new_person"

# per-person option keys (suffixes after "<person_entity_id>|")
PERSON_FIELDS = ("room", "heart_rate", "breathing", "battery", "activity")
