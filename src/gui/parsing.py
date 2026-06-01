from __future__ import annotations

import math
from typing import Optional


def parse_float(value: str, field_name: str) -> float:
    text = value.strip().replace(",", ".")
    if not text:
        raise ValueError(f"{field_name}: введите число")
    try:
        number = float(text)
    except ValueError as error:
        raise ValueError(f"{field_name}: некорректное число") from error
    if not math.isfinite(number):
        raise ValueError(f"{field_name}: некорректное число")
    return number


def parse_tilt_deg(value: str, field_name: str) -> float:
    text = value.strip().replace(",", ".")
    if not text:
        return 0.0
    number = parse_float(text, field_name)
    if number <= -90.0 or number >= 90.0:
        raise ValueError(f"{field_name}: угол должен быть в диапазоне -89…89")
    return number


def parse_optional_float(value: str, field_name: str) -> float:
    text = value.strip().replace(",", ".")
    if not text:
        return 0.0
    return parse_float(text, field_name)


def parse_optional_non_negative_float(value: str, field_name: str) -> float:
    number = parse_optional_float(value, field_name)
    if number < 0.0:
        raise ValueError(f"{field_name}: должно быть >= 0")
    return number


def parse_positive_float(value: str, field_name: str) -> float:
    number = parse_float(value, field_name)
    if number <= 0:
        raise ValueError(f"{field_name}: должно быть больше 0")
    return number


def parse_positive_int(value: str, field_name: str) -> int:
    text = value.strip()
    try:
        number = int(text)
    except ValueError as error:
        raise ValueError(f"{field_name}: введите целое число") from error
    if number < 1:
        raise ValueError(f"{field_name}: должно быть >= 1")
    return number


def parse_optional_angle(value: str, field_name: str) -> Optional[float]:
    text = value.strip().replace(",", ".")
    if not text:
        return None
    number = parse_float(text, field_name)
    if number < 0.0 or number >= 360.0:
        raise ValueError(f"{field_name}: угол должен быть в диапазоне 0–360")
    return number


def format_number(value: float) -> str:
    if float(value).is_integer():
        return str(int(value))
    return f"{value:g}"
