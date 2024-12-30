export function getVisibleSection(sections: (HTMLElement | null)[]) {
  return sections.reduce(
    (acc, section) => {
      if (section) {
        const rect = section.getBoundingClientRect();
        const visibleHeight = Math.max(
          0,
          Math.min(rect.bottom, window.innerHeight) - Math.max(0, rect.top)
        );
        const visibleWidth = Math.max(
          0,
          Math.min(rect.right, window.innerWidth) - Math.max(0, rect.left)
        );
        const visibleRatio =
          (visibleHeight * visibleWidth) / (rect.width * rect.height);
        if (visibleRatio > acc.maxRatio) {
          acc.maxRatio = visibleRatio;
          acc.section = section;
        }
      }
      return acc;
    },
    { maxRatio: 0, section: null as HTMLElement | null }
  ).section;
}

export function createSlug(title: string): string {
  return title
    .toLowerCase() // Convert to lowercase
    .replace(/\s+/g, '-') // Replace spaces with hyphens
    .replace(/[^\w\-]+/g, ''); // Remove non-alphanumeric characters except hyphens
}