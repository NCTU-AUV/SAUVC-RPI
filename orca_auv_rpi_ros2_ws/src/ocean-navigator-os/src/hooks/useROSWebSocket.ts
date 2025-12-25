import { useState, useEffect, useCallback, useRef } from 'react';

type WebSocketConfig = {
    url: string;
    reconnectInterval?: number;
};

type ROSMessage = {
    type: 'topic' | 'action';
    data: {
        topic_name?: string;
        action_name?: string;
        msg?: any;
        [key: string]: any;
    };
};

export const useROSWebSocket = ({ url, reconnectInterval = 3000 }: WebSocketConfig) => {
    const [isConnected, setIsConnected] = useState(false);
    const [lastMessage, setLastMessage] = useState<any>(null);
    const ws = useRef<WebSocket | null>(null);
    const reconnectTimeout = useRef<NodeJS.Timeout>();

    const connect = useCallback(() => {
        try {
            const socket = new WebSocket(url);

            socket.onopen = () => {
                console.log('ROS WebSocket Connected');
                setIsConnected(true);
            };

            socket.onclose = () => {
                console.log('ROS WebSocket Disconnected');
                setIsConnected(false);
                // Attempt reconnect
                reconnectTimeout.current = setTimeout(() => {
                    connect();
                }, reconnectInterval);
            };

            socket.onerror = (error) => {
                console.error('ROS WebSocket Error:', error);
                socket.close();
            };

            socket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    setLastMessage(data);
                } catch (e) {
                    console.warn('Received non-JSON message:', event.data);
                }
            };

            ws.current = socket;
        } catch (error) {
            console.error('Connection failed:', error);
            reconnectTimeout.current = setTimeout(() => {
                connect();
            }, reconnectInterval);
        }
    }, [url, reconnectInterval]);

    useEffect(() => {
        connect();

        return () => {
            if (ws.current) {
                ws.current.close();
            }
            if (reconnectTimeout.current) {
                clearTimeout(reconnectTimeout.current);
            }
        };
    }, [connect]);

    const sendJsonMessage = useCallback((message: ROSMessage) => {
        if (ws.current && ws.current.readyState === WebSocket.OPEN) {
            ws.current.send(JSON.stringify(message));
        } else {
            console.warn('WebSocket is not connected. Message not sent:', message);
        }
    }, []);

    return { isConnected, lastMessage, sendJsonMessage };
};
