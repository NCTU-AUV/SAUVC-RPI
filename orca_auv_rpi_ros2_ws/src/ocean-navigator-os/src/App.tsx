import { Toaster } from "@/components/ui/toaster";
import { Toaster as Sonner } from "@/components/ui/sonner";
import { TooltipProvider } from "@/components/ui/tooltip";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import { SidebarProvider } from "@/components/ui/sidebar";
import { AppSidebar } from "@/components/AppSidebar";
import { DemoModeProvider } from "@/context/DemoModeContext";
import { ThemeProvider } from "@/context/ThemeContext";
import Index from "./pages/Index";
import MissionSetup from "./pages/MissionSetup";
import Dashboard from "./pages/Dashboard";
import ManualControl from "./pages/ManualControl";
import MissionManager from "./pages/MissionManager";
import SystemSettings from "./pages/SystemSettings";
import NotFound from "./pages/NotFound";

const queryClient = new QueryClient();

const App = () => (
  <QueryClientProvider client={queryClient}>
    <TooltipProvider>
      <Toaster />
      <Sonner />
      <ThemeProvider>
        <DemoModeProvider>
          <BrowserRouter>
            <SidebarProvider>
              <div className="flex min-h-screen w-full">
                <AppSidebar />
                <main className="flex-1">
                  <Routes>
                    <Route path="/" element={<Index />} />
                    <Route path="/mission-setup" element={<MissionSetup />} />
                    <Route path="/dashboard" element={<Dashboard />} />
                    <Route path="/manual-control" element={<ManualControl />} />
                    <Route path="/mission-manager" element={<MissionManager />} />
                    <Route path="/system-settings" element={<SystemSettings />} />
                    <Route path="*" element={<NotFound />} />
                  </Routes>
                </main>
              </div>
            </SidebarProvider>
          </BrowserRouter>
        </DemoModeProvider>
      </ThemeProvider>
    </TooltipProvider>
  </QueryClientProvider>
);

export default App;
