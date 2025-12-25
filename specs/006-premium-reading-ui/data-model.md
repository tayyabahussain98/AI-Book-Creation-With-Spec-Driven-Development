# Data Model: CSS Custom Properties Design Tokens

This document defines the design tokens for the Premium Reading UI using CSS custom properties.

## Color Palette (NO BLUE)

### Primary Colors

| Token | Value | Usage |
|-------|-------|-------|
| `--color-primary` | `#26A69A` | Primary buttons, links, accents |
| `--color-primary-hover` | `#1E8A7E` | Hover states |
| `--color-primary-light` | `#3DB8AB` | Light backgrounds, subtle accents |
| `--color-primary-lighter` | `#4FC0B0` | Hover backgrounds, borders |
| `--color-primary-lightest` | `#7DD9C8` | Active states, highlights |

### Accent Colors

| Token | Value | Usage |
|-------|-------|-------|
| `--color-accent` | `#FFC857` | Warm highlights, calls-to-action |
| `--color-accent-hover` | `#E6B34E` | Hover state for accent elements |

### Light Mode Backgrounds

| Token | Value | Usage |
|-------|-------|-------|
| `--color-background` | `#F8FAFC` | Main page background |
| `--color-surface` | `#FFFFFF` | Card backgrounds, content areas |
| `--color-surface-hover` | `#F1F5F9` | Hover backgrounds |
| `--color-border` | `#E2E8F0` | Borders, dividers |

### Light Mode Text

| Token | Value | Usage |
|-------|-------|-------|
| `--color-text-primary` | `#1F2937` | Headings, primary text |
| `--color-text-secondary` | `#4B5563` | Body text, descriptions |
| `--color-text-muted` | `#9CA3AF` | Metadata, captions |

### Dark Mode Backgrounds

| Token | Value | Usage |
|-------|-------|-------|
| `--color-background-dark` | `#0B1220` | Main page background (dark) |
| `--color-surface-dark` | `#1A2332` | Card backgrounds (dark) |
| `--color-surface-hover-dark` | `#243043` | Hover backgrounds (dark) |
| `--color-border-dark` | `#374151` | Borders (dark) |

### Dark Mode Text

| Token | Value | Usage |
|-------|-------|-------|
| `--color-text-primary-dark` | `#F3F4F6` | Headings (dark) |
| `--color-text-secondary-dark` | `#D1D5DB` | Body text (dark) |
| `--color-text-muted-dark` | `#9CA3AF` | Metadata (dark) |

## Typography

### Font Families

| Token | Value | Usage |
|-------|-------|-------|
| `--font-family-base` | `'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif` | Body text |
| `--font-family-heading` | `'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif` | Headings |
| `--font-family-mono` | `'SF Mono', 'Fira Code', Menlo, monospace` | Code blocks |

### Font Sizes

| Token | Value | Rem | Usage |
|-------|-------|-----|-------|
| `--font-size-xs` | `0.75rem` | 12px | Metadata, captions |
| `--font-size-sm` | `0.875rem` | 14px | Small text |
| `--font-size-base` | `1rem` | 16px | Body text |
| `--font-size-lg` | `1.125rem` | 18px | Large body |
| `--font-size-xl` | `1.25rem` | 20px | Section headers |
| `--font-size-2xl` | `1.5rem` | 24px | Page headings |
| `--font-size-3xl` | `1.875rem` | 30px | Hero title |
| `--font-size-4xl` | `2.25rem` | 36px | Hero headline |

### Font Weights

| Token | Value | Usage |
|-------|-------|-------|
| `--font-weight-normal` | `400` | Body text |
| `--font-weight-medium` | `500` | Semi-bold body |
| `--font-weight-semibold` | `600` | Subheadings |
| `--font-weight-bold` | `700` | Headings |

### Line Heights

| Token | Value | Usage |
|-------|-------|-------|
| `--line-height-tight` | `1.25` | Headings |
| `--line-height-normal` | `1.6` | Body text |
| `--line-height-relaxed` | `1.75` | Long-form content |

## Spacing Scale

| Token | Value | Pixels | Usage |
|-------|-------|--------|-------|
| `--spacing-1` | `0.25rem` | 4px | Tight spacing |
| `--spacing-2` | `0.5rem` | 8px | Inline spacing |
| `--spacing-3` | `0.75rem` | 12px | Small padding |
| `--spacing-4` | `1rem` | 16px | Base unit |
| `--spacing-5` | `1.25rem` | 20px | Medium padding |
| `--spacing-6` | `1.5rem` | 24px | Section padding |
| `--spacing-8` | `2rem` | 32px | Large padding |
| `--spacing-10` | `2.5rem` | 40px | XL padding |
| `--spacing-12` | `3rem` | 48px | XXL padding |
| `--spacing-16` | `4rem` | 64px | Section gaps |

## Border Radius

| Token | Value | Usage |
|-------|-------|-------|
| `--border-radius-sm` | `0.25rem` | Small elements, badges |
| `--border-radius-md` | `0.5rem` | Buttons, inputs |
| `--border-radius-lg` | `0.75rem` | Cards, modals |
| `--border-radius-xl` | `1rem` | Large cards |
| `--border-radius-full` | `9999px` | Pills, circular |

## Shadows

| Token | Value | Usage |
|-------|-------|-------|
| `--shadow-sm` | `0 1px 2px 0 rgb(0 0 0 / 0.05)` | Subtle depth |
| `--shadow-md` | `0 4px 6px -1px rgb(0 0 0 / 0.1), 0 2px 4px -2px rgb(0 0 0 / 0.1)` | Cards, dropdowns |
| `--shadow-lg` | `0 10px 15px -3px rgb(0 0 0 / 0.1), 0 4px 6px -4px rgb(0 0 0 / 0.1)` | Elevated elements |
| `--shadow-xl` | `0 20px 25px -5px rgb(0 0 0 / 0.1), 0 8px 10px -6px rgb(0 0 0 / 0.1)` | Hero, modals |

## Transitions

| Token | Value | Usage |
|-------|-------|-------|
| `--transition-fast` | `150ms ease` | Hover, focus states |
| `--transition-normal` | `250ms ease` | Button clicks, reveals |
| `--transition-slow` | `350ms ease` | Large animations |

## Docusaurus Infima Overrides

```css
:root {
  /* Override Docusaurus Infima variables */
  --ifm-color-primary: var(--color-primary);
  --ifm-color-primary-dark: var(--color-primary-hover);
  --ifm-color-primary-darker: var(--color-primary-hover);
  --ifm-color-primary-darkest: var(--color-primary-hover);
  --ifm-color-primary-light: var(--color-primary-light);
  --ifm-color-primary-lighter: var(--color-primary-lighter);
  --ifm-color-primary-lightest: var(--color-primary-lightest);
  --ifm-code-font-size: 95%;
  --ifm-font-family-base: var(--font-family-base);
  --ifm-heading-font-weight: var(--font-weight-bold);
  --ifm-navbar-height: 4rem;
  --ifm-navbar-shadow: var(--shadow-md);
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.1);
}

[data-theme='dark'] {
  --ifm-color-primary: var(--color-primary);
  --ifm-color-primary-dark: var(--color-primary-hover);
  --ifm-color-primary-darker: var(--color-primary-hover);
  --ifm-color-primary-darkest: var(--color-primary-hover);
  --ifm-color-primary-light: var(--color-primary-light);
  --ifm-color-primary-lighter: var(--color-primary-lighter);
  --ifm-color-primary-lightest: var(--color-primary-lightest);
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.3);
}
```
