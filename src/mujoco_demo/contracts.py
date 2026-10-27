"""Validated input contract between the language planner and task executor."""

from pydantic import BaseModel, ConfigDict, Field


class PickPlaceTask(BaseModel):
    """A high-level request; scene resolution and motion remain deterministic."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    object_name: str = Field(min_length=1)
    target_name: str = Field(min_length=1)
