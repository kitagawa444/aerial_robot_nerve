#!/usr/bin/env python3
"""Generate the shared Spinal Link DDS topic table without third-party YAML."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List


def scalar(value: str):
    value = value.strip()
    if value in ("true", "false"):
        return value == "true"
    try:
        return int(value)
    except ValueError:
        return value.strip('"')


def load_topics(path: Path) -> List[Dict[str, object]]:
    topics: List[Dict[str, object]] = []
    current: Dict[str, object] | None = None
    in_topics = False
    for line_number, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        stripped = line.strip()
        if stripped == "topics:":
            in_topics = True
            continue
        if not in_topics:
            continue
        if stripped.startswith("- "):
            if current is not None:
                topics.append(current)
            current = {}
            stripped = stripped[2:]
        if current is None or ":" not in stripped:
            raise ValueError(f"{path}:{line_number}: invalid topic entry")
        key, value = stripped.split(":", 1)
        current[key.strip()] = scalar(value)
    if current is not None:
        topics.append(current)
    return topics


def validate(topics: List[Dict[str, object]]) -> None:
    required = {
        "message",
        "id",
        "topic",
        "direction",
        "enabled",
        "reliability",
        "durability",
        "history_depth",
        "application_ack",
        "deduplicate",
    }
    seen_messages = set()
    seen_ids = set()
    seen_names = set()
    for topic in topics:
        missing = required - topic.keys()
        if missing:
            raise ValueError(f"{topic.get('message', '<unknown>')}: missing {sorted(missing)}")
        message = str(topic["message"])
        if message in seen_messages or topic["id"] in seen_ids or topic["topic"] in seen_names:
            raise ValueError(f"duplicate message, id, or topic at {message}")
        seen_messages.add(message)
        seen_ids.add(topic["id"])
        seen_names.add(topic["topic"])
        if topic["direction"] not in ("spinal_to_host", "host_to_spinal"):
            raise ValueError(f"{message}: invalid direction")
        if topic["reliability"] not in ("best_effort", "reliable"):
            raise ValueError(f"{message}: invalid reliability")
        if topic["durability"] not in ("volatile", "transient_local"):
            raise ValueError(f"{message}: invalid durability")
        if not 1 <= int(topic["history_depth"]) <= 255:
            raise ValueError(f"{message}: history_depth must be 1..255")
        if topic["application_ack"]:
            if topic["direction"] != "host_to_spinal" or topic["reliability"] != "reliable":
                raise ValueError(f"{message}: ACK requires reliable host_to_spinal topic")
            if int(topic.get("ack_timeout_ms", 0)) <= 0:
                raise ValueError(f"{message}: ACK timeout is required")
        if topic["deduplicate"] and not topic["application_ack"]:
            raise ValueError(f"{message}: deduplication requires ACK")


def cpp_bool(value: object) -> str:
    return "true" if value else "false"


def generate(topics: List[Dict[str, object]], source: Path) -> str:
    rows = []
    asserts = []
    for index, topic in enumerate(topics, 1):
        direction = "SPINAL_TO_HOST" if topic["direction"] == "spinal_to_host" else "HOST_TO_SPINAL"
        reliability = "RELIABLE" if topic["reliability"] == "reliable" else "BEST_EFFORT"
        durability = "TRANSIENT_LOCAL" if topic["durability"] == "transient_local" else "VOLATILE"
        rows.append(
            "  {MessageId::%s, \"%s\", TopicDirection::%s, Reliability::%s, "
            "TopicDurability::%s, %dU, %s, %s, %s, %dU, %dU, %dU},"
            % (
                topic["message"], topic["topic"], direction, reliability,
                durability, int(topic["history_depth"]), cpp_bool(topic["enabled"]),
                cpp_bool(topic["application_ack"]), cpp_bool(topic["deduplicate"]),
                int(topic.get("ack_timeout_ms", 0)), int(topic.get("max_retries", 0)), index,
            )
        )
        asserts.append(
            "static_assert(static_cast<uint16_t>(MessageId::%s) == %dU, \"Message ID mismatch\");"
            % (topic["message"], int(topic["id"]))
        )
    return """// Generated from %s. Do not edit manually.
#pragma once

#include <cstddef>
#include <cstdint>

#include "communication/spinal_link_protocol.h"

namespace aerial {
namespace vehicle {

enum class TopicDirection : uint8_t { SPINAL_TO_HOST = 0, HOST_TO_SPINAL = 1 };
enum class TopicDurability : uint8_t { VOLATILE = 0, TRANSIENT_LOCAL = 1 };

struct TopicDescriptor {
  MessageId message_id;
  const char *topic_name;
  TopicDirection direction;
  Reliability reliability;
  TopicDurability durability;
  uint8_t history_depth;
  bool enabled;
  bool application_ack;
  bool deduplicate;
  uint16_t ack_timeout_ms;
  uint8_t max_retries;
  uint16_t entity_id;
};

// clang-format off
constexpr TopicDescriptor kSpinalLinkTopics[] = {
%s
};
constexpr size_t kSpinalLinkTopicCount =
    sizeof(kSpinalLinkTopics) / sizeof(kSpinalLinkTopics[0]);

inline const TopicDescriptor *topicDescriptor(MessageId message_id) {
  for (size_t index = 0; index < kSpinalLinkTopicCount; ++index) {
    if (kSpinalLinkTopics[index].message_id == message_id)
      return &kSpinalLinkTopics[index];
  }
  return nullptr;
}

inline const TopicDescriptor *enabledTopicDescriptor(MessageId message_id) {
  const TopicDescriptor *descriptor = topicDescriptor(message_id);
  return descriptor != nullptr && descriptor->enabled ? descriptor : nullptr;
}

%s
// clang-format on

} // namespace vehicle
} // namespace aerial
""" % (source.name, "\n".join(rows), "\n".join(asserts))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    topics = load_topics(args.config)
    validate(topics)
    output = generate(topics, args.config)
    if args.check:
        if not args.output.exists() or args.output.read_text() != output:
            raise SystemExit(f"generated file is stale: {args.output}")
        return
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output)


if __name__ == "__main__":
    main()
