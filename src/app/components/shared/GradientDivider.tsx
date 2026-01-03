/**
 * Props for the GradientDivider component.
 */
interface GradientDividerProps {
  /**
   * If true, uses the accent color (var(--colored-border)) for the gradient.
   * Otherwise, uses the foreground color (var(--foreground)).
   * @default false
   */
  color?: boolean;
}

/**
 * A horizontal divider with a gradient effect.
 * The gradient fades from the background color, to a central color, and back to the background color.
 * The central color can be either the standard foreground or an accent color based on the `color` prop.
 * @param props - The properties for the GradientDivider.
 * @returns A span element styled as a gradient divider.
 */
export function GradientDivider(props: GradientDividerProps) {
    const centralColor = props.color ? "var(--colored-border)" : "var(--foreground)";
    return (
        <span
            className={`inline-block w-full h-[2px] relative bg-gradient-to-r from-[var(--background)] via-[${centralColor}] to-[var(--background)]`}
        ></span>
    );
}
