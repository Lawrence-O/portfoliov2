export function GradientDivider(props: {color?: boolean}) {
    if (!props.color) {
        return <span className={`inline-block w-full relative after:content-[''] after:absolute after:left-0 after:bottom-0 after:w-full after:h-[2px] after:bg-gradient-to-r after:from-[var(--background)] after:via-[var(--foreground)] after:to-[var(--background)]`}></span>
    } else {
        return <span className={`inline-block w-full relative after:content-[''] after:absolute after:left-0 after:bottom-0 after:w-full after:h-[2px] after:bg-gradient-to-r after:from-[var(--background)] after:via-[var(--colored-border)] after:to-[var(--background)]`}></span>
    }
}