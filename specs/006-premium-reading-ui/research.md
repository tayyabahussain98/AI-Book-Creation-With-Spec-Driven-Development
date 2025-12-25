# Research: Premium Reading UI Implementation

## Docusaurus Theme Customization

### Navbar Logo Configuration

**Decision**: Use `docusaurus.config.ts` navbar.logo configuration with custom SVG logo

**Rationale**: Docusaurus supports SVG logos natively, allows alt text, and handles dark mode variants automatically via class-based src attributes.

**Implementation**:
```typescript
// docusaurus.config.ts
navbar: {
  logo: {
    alt: 'Physical AI Book Logo',
    src: 'img/logo.svg',
  },
}
```

### Favicon Configuration

**Decision**: Configure favicon in `docusaurus.config.ts` with path to `static/img/favicon.ico`

**Rationale**: Docusaurus reads favicon from static directory. An ICO file ensures broad browser compatibility.

**Implementation**:
```typescript
// docusaurus.config.ts
favicon: 'img/favicon.ico',
```

### CSS Custom Properties in Docusaurus

**Decision**: Override Infima CSS variables in `:root` and `[data-theme='dark']` in `src/css/custom.css`

**Rationale**: Docusaurus uses Infima for styling. CSS custom properties are the recommended override mechanism. No JavaScript required.

**Implementation**:
```css
:root {
  --ifm-color-primary: #26A69A;
  --ifm-color-primary-dark: #1E8A7E;
  --ifm-color-primary-darker: #1C8175;
  --ifm-color-primary-darkest: #186B61;
  --ifm-color-primary-light: #3DB8AB;
  --ifm-color-primary-lighter: #4FC0B0;
  --ifm-color-primary-lightest: #7DD9C8;
  --ifm-background-color: #F8FAFC;
  --ifm-font-color-base: #1F2937;
}

[data-theme='dark'] {
  --ifm-color-primary: #26A69A;
  --ifm-background-color: #0B1220;
  --ifm-font-color-base: #E5E7EB;
}
```

## SVG Icon Optimization

**Decision**: Use inline SVG icons in React components, optimized SVGO style

**Rationale**: Inline SVGs allow CSS styling via fill/currentColor, eliminate network requests, and enable tree-shaking.

**Optimization approach**:
- Remove XML declarations, comments, unused namespaces
- Simplify paths, use relative coordinates
- Set `fill="currentColor"` for CSS color control
- Keep under 2KB per icon (4 icons = <8KB total)

## Sidebar Styling in Docusaurus

**Decision**: Target Docusaurus sidebar classes with CSS overrides

**Classes to override**:
- `.theme-doc-sidebar-container` - Sidebar container
- `.menu__link` - Sidebar navigation links
- `.menu__link--active` - Active link state
- `.menu__list-item` - List items

**Implementation**:
```css
.sidebar {
  background-color: var(--color-surface);
}

.menu__link {
  border-radius: var(--border-radius-md);
  color: var(--color-text-secondary);
  transition: all var(--transition-fast);
}

.menu__link--active {
  background-color: var(--color-primary-lightest);
  color: var(--color-primary);
  border-left: 3px solid var(--color-primary);
}
```

## Mobile Responsiveness

**Decision**: Use CSS Grid with auto-fit for module cards, standard media queries for breakpoints

**Breakpoints**:
- Desktop: Multi-column grid (auto-fit, minmax 280px)
- Tablet: 2-column grid
- Mobile: Single column stack (320px-428px)

**Implementation**:
```css
.module-cards {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: var(--spacing-6);
}

@media (max-width: 768px) {
  .module-cards {
    grid-template-columns: 1fr;
  }
}
```

## Alternatives Considered

### 1. Custom Docusaurus Theme vs Classic Theme Swizzling

**Rejected**: Creating a custom theme or swizzling components

**Reason**: Over-engineering for UI changes. Classic theme + CSS custom properties provides sufficient flexibility for all requirements.

### 2. External CSS Framework vs Custom CSS

**Rejected**: Bootstrap, Tailwind, or other frameworks

**Reason**: Docusaurus already bundles Infima. Adding another framework increases bundle size and conflicts with existing styles.

### 3. Image Formats

**Rejected**: PNG or JPG for icons

**Reason**: SVG offers infinite scalability, smaller file sizes, and CSS color control. Required for the "no blue" palette constraint.

## References

- Docusaurus Theming: https://docusaurus.io/docs/styling/layout
- Infima CSS Variables: https://infima.dev/docs/
- CSS Custom Properties: https://developer.mozilla.org/en-US/docs/Web/CSS/Using_CSS_custom_properties
