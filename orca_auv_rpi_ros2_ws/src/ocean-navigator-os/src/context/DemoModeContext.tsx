import { createContext, useContext, useState, ReactNode } from "react";

interface DemoModeContextType {
  demoMode: boolean;
  setDemoMode: (value: boolean) => void;
  testMode: boolean;
  setTestMode: (value: boolean) => void;
}

const DemoModeContext = createContext<DemoModeContextType | undefined>(undefined);

export const DemoModeProvider = ({ children }: { children: ReactNode }) => {
  const [demoMode, setDemoMode] = useState(false);
  const [testMode, setTestMode] = useState(false);

  return (
    <DemoModeContext.Provider value={{ demoMode, setDemoMode, testMode, setTestMode }}>
      {children}
    </DemoModeContext.Provider>
  );
};

export const useDemoMode = () => {
  const context = useContext(DemoModeContext);
  if (context === undefined) {
    throw new Error("useDemoMode must be used within a DemoModeProvider");
  }
  return context;
};
