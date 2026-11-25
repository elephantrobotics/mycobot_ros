"""Feature-scoped SDK-side flange/tool conversion; no controller TCP writes."""
import numpy as np


class ToolReferenceContext:
    def __init__(self, sdk):
        self.sdk = sdk
        self.saved = None

    def apply(self, with_gripper):
        if self.saved is not None:
            raise RuntimeError('Tool coordinate context is already active')
        matrix = self.sdk.set_tool_reference(np.array([0., 0., 165. if with_gripper else 0., 0., 0., 0.]))
        self.saved = (self.sdk.tool_matrix.copy(), self.sdk._tool_control)
        self.sdk.tool_matrix = matrix
        self.sdk._tool_control = with_gripper

    def restore(self):
        if self.saved is not None:
            self.sdk.tool_matrix, self.sdk._tool_control = self.saved
            self.saved = None
