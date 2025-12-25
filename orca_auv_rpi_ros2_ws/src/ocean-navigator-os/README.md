# Ocean Navigator OS

A modern web application built with React, Vite, and TypeScript. This project features a rich user interface using shadcn/ui and Tailwind CSS, with capabilities for mapping (Leaflet), 3D visualization (Three.js), and data charting (Recharts).

## Tech Stack

- **Framework:** [React](https://react.dev/)
- **Build Tool:** [Vite](https://vitejs.dev/)
- **Language:** [TypeScript](https://www.typescriptlang.org/)
- **Styling:** [Tailwind CSS](https://tailwindcss.com/)
- **UI Components:** [shadcn/ui](https://ui.shadcn.com/)
- **Maps:** [React Leaflet](https://react-leaflet.js.org/)
- **3D Graphics:** [React Three Fiber](https://docs.pmnd.rs/react-three-fiber)
- **Charts:** [Recharts](https://recharts.org/)
- **State Management:** [TanStack Query](https://tanstack.com/query/latest)

## Getting Started

### Prerequisites

Ensure you have Node.js installed on your machine.

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   ```

2. Navigate to the project directory:
   ```bash
   cd ocean-navigator-os
   ```

3. Install dependencies:
   ```bash
   npm install
   ```

### Development

To start the development server:

```bash
npm run dev
```

The application will be available at `http://localhost:8080` (or the port shown in your terminal).

### Building for Production

To build the application for production:

```bash
npm run build
```

To preview the production build locally:

```bash
npm run preview
```

## Project Structure

- `src/` - Source code
  - `components/` - React components
  - `pages/` - Page components
  - `hooks/` - Custom React hooks
  - `lib/` - Utility functions and configurations
