"""Display rounded zero without a sign; leave underlying values untouched."""


def display_value(value, decimals):
    return 0.0 if round(value, decimals) == 0 else value


def format_fixed(value, decimals):
    return f'{display_value(value, decimals):.{decimals}f}'


class PositiveZeroDisplay:
    """Mixin for either Qt binding's QDoubleSpinBox."""

    def textFromValue(self, value):
        return super().textFromValue(display_value(value, self.decimals()))
