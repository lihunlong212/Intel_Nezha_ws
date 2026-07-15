from __future__ import annotations

from collections.abc import Mapping


VALID_DEVICE_IDS = ("drone1", "drone2", "drone3")
PICKUP_RELEASE_STATES = frozenset({"DELIVERING", "DELIVERED", "LANDED"})
DELIVERY_RELEASE_STATES = frozenset({"DELIVERED", "LANDED"})


def device_priority(device_id: str) -> int:
    normalized = device_id.strip().lower()
    if normalized not in VALID_DEVICE_IDS:
        raise ValueError(f"device_id must be one of {VALID_DEVICE_IDS}, got {device_id!r}")
    return int(normalized[-1])


def default_transit_y_cm(device_id: str) -> float:
    priority = device_priority(device_id)
    return -186.0 if priority == 2 else 186.0


def select_predecessor(
    device_id: str,
    peer_status: Mapping[str, Mapping[str, object]],
) -> str | None:
    """Return the nearest higher-priority peer that consumed an order."""
    priority = device_priority(device_id)
    for candidate_priority in range(priority - 1, 0, -1):
        candidate = f"drone{candidate_priority}"
        payload = peer_status.get(candidate)
        if payload is not None and payload.get("available") is False:
            return candidate
    return None


def predecessor_reached(
    predecessor_id: str | None,
    peer_status: Mapping[str, Mapping[str, object]],
    allowed_states: set[str] | frozenset[str],
) -> bool:
    if predecessor_id is None:
        return True
    state = str(peer_status.get(predecessor_id, {}).get("current_state") or "")
    return state in allowed_states
