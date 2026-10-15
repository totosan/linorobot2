"""Small fail-closed consumer shared by mock and vision-off backends."""

from dataclasses import dataclass
from typing import Optional, Tuple

from .contract import (
    SCHEMA_VERSION,
    VisionClockDomainError,
    VisionContractError,
    VisionFutureResultError,
    VisionStaleResultError,
    require_fresh_result,
    validate_response,
)


@dataclass(frozen=True)
class ConsumerOutcome:
    """Normalized consumer result; deliberately contains no robot command."""

    state: str
    detections: Tuple[dict, ...] = ()
    age_ns: Optional[int] = None
    error_code: Optional[str] = None
    message: Optional[str] = None


class VisionOffBackend:
    """Backend representing an intentionally disabled vision source."""

    def get_response(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "status": "unavailable",
            "result": None,
            "error": {
                "code": "vision_disabled",
                "message": "Vision backend is disabled by configuration.",
            },
        }


class VisionResultConsumer:
    """Consume any get_response() backend with freshness and clock checks."""

    def __init__(self, backend, max_age_ns):
        if isinstance(max_age_ns, bool) or not isinstance(max_age_ns, int) or max_age_ns < 0:
            raise ValueError("max_age_ns must be a nonnegative integer")
        if not callable(getattr(backend, "get_response", None)):
            raise TypeError("backend must provide get_response()")
        self.backend = backend
        self.max_age_ns = max_age_ns

    def consume(self, now_ns, clock_domain):
        """Return a safe status instead of stale detections or backend exceptions."""
        try:
            response = self.backend.get_response()
        except Exception as exc:
            return ConsumerOutcome(
                "error", error_code="backend_exception",
                message=f"{type(exc).__name__}: {exc}")

        try:
            response = validate_response(response)
        except VisionContractError as exc:
            return ConsumerOutcome(
                "error", error_code="invalid_response", message=str(exc))

        if response["status"] != "ok":
            return ConsumerOutcome(
                response["status"], error_code=response["error"]["code"],
                message=response["error"]["message"])

        try:
            age_ns = require_fresh_result(
                response, now_ns, clock_domain, self.max_age_ns)
        except VisionStaleResultError as exc:
            return ConsumerOutcome("stale", error_code="stale_result", message=str(exc))
        except VisionFutureResultError as exc:
            return ConsumerOutcome("future", error_code="future_result", message=str(exc))
        except VisionClockDomainError as exc:
            return ConsumerOutcome("clock_mismatch", error_code="clock_domain_mismatch",
                                   message=str(exc))
        except VisionContractError as exc:
            return ConsumerOutcome("error", error_code="invalid_time_context", message=str(exc))

        detections = tuple(response["result"]["detections"])
        return ConsumerOutcome("ok" if detections else "empty",
                               detections=detections, age_ns=age_ns)
