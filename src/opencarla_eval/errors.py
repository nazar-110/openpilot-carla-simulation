"""Project-specific exceptions with user-facing messages."""


class EvaluationError(RuntimeError):
  """Base class for expected evaluation failures."""


class ConfigurationError(EvaluationError):
  """Raised when a scenario or experiment configuration is invalid."""


class DependencyUnavailableError(EvaluationError):
  """Raised when an optional simulator dependency is unavailable."""


class SimulatorConnectionError(EvaluationError):
  """Raised when a CARLA server cannot be reached or initialized."""


class StaleResultError(EvaluationError):
  """Raised when a completed run does not match the requested provenance."""
