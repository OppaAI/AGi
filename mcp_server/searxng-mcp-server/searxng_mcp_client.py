"""
SearXNG MCP Client
Python client for custom MCP SearXNG server
"""

import json
import subprocess
import threading
import queue
from pathlib import Path
from typing import Optional, Dict, List, Any


class SearXNGMCPClient:
    """
    Client for custom MCP SearXNG server
    
    Features:
    - Full content extraction from web pages
    - Clean markdown formatting
    - Multiple search strategies
    - Automatic error handling and fallback
    """
    
    def __init__(self, server_path: str = "searxng-mcp-server", logger=None):
        """
        Initialize MCP client
        
        Args:
            server_path: Path to the MCP server executable
            logger: Optional logger instance
        """
        self.server_path = server_path
        self.logger = logger
        self.process = None
        self.output_queue = queue.Queue()
        self.request_id = 0
        self._reader_thread = None
    
    def _log(self, level: str, message: str):
        """Internal logging helper"""
        if self.logger:
            if level == "info":
                self.logger.info(message)
            elif level == "error":
                self.logger.error(message)
            elif level == "warn":
                self.logger.warn(message)
        else:
            print(f"[{level.upper()}] {message}")
    
    def start(self) -> bool:
        """Start the MCP server process"""
        try:
            self.process = subprocess.Popen(
                [self.server_path],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            # Start reader thread
            self._reader_thread = threading.Thread(
                target=self._read_responses,
                daemon=True
            )
            self._reader_thread.start()
            
            self._log("info", "✅ SearXNG MCP server started")
            return True
            
        except Exception as e:
            self._log("error", f"❌ Failed to start MCP server: {e}")
            return False
    
    def _read_responses(self):
        """Read responses from server in background thread"""
        while self.process and self.process.poll() is None:
            try:
                line = self.process.stdout.readline()
                if line:
                    response = json.loads(line)
                    self.output_queue.put(response)
            except json.JSONDecodeError:
                # Skip non-JSON lines (like debug output)
                continue
            except Exception as e:
                self._log("error", f"Reader thread error: {e}")
                break
    
    def _send_request(self, method: str, params: Dict) -> Optional[Dict]:
        """Send request and wait for response"""
        if not self.process or self.process.poll() is not None:
            self._log("error", "MCP server not running")
            return None
        
        self.request_id += 1
        request = {
            "jsonrpc": "2.0",
            "id": self.request_id,
            "method": method,
            "params": params
        }
        
        try:
            # Send request
            self.process.stdin.write(json.dumps(request) + '\n')
            self.process.stdin.flush()
            
            # Wait for response (with timeout)
            import time
            timeout = 30  # 30 seconds
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                try:
                    response = self.output_queue.get(timeout=1)
                    if response.get('id') == self.request_id:
                        return response
                except queue.Empty:
                    continue
            
            self._log("error", "Request timeout")
            return None
            
        except Exception as e:
            self._log("error", f"Request failed: {e}")
            return None
    
    def search_web(
        self, 
        query: str, 
        num_results: int = 10,
        fetch_content: bool = True
    ) -> Optional[Dict]:
        """
        Search the web with full content extraction
        
        Args:
            query: Search query
            num_results: Number of results (max 20)
            fetch_content: Whether to fetch full page content
            
        Returns:
            Dict with search results or None on error
        """
        response = self._send_request("tools/call", {
            "name": "search_web",
            "arguments": {
                "query": query,
                "num_results": min(num_results, 20),
                "fetch_content": fetch_content
            }
        })
        
        if response and 'result' in response:
            return self._parse_search_response(response['result'])
        
        return None
    
    def search_snippets_only(
        self,
        query: str,
        num_results: int = 10
    ) -> Optional[Dict]:
        """
        Fast search returning only snippets
        
        Args:
            query: Search query
            num_results: Number of results
            
        Returns:
            Dict with search results or None on error
        """
        response = self._send_request("tools/call", {
            "name": "search_snippets_only",
            "arguments": {
                "query": query,
                "num_results": num_results
            }
        })
        
        if response and 'result' in response:
            return self._parse_search_response(response['result'])
        
        return None
    
    def fetch_url(self, url: str) -> Optional[str]:
        """
        Fetch and extract content from specific URL
        
        Args:
            url: URL to fetch
            
        Returns:
            Extracted content or None on error
        """
        response = self._send_request("tools/call", {
            "name": "fetch_url",
            "arguments": {
                "url": url
            }
        })
        
        if response and 'result' in response:
            content = response['result'].get('content', [])
            if content and len(content) > 0:
                return content[0].get('text', '')
        
        return None
    
    def _parse_search_response(self, result: Dict) -> Dict:
        """Parse search response into structured format"""
        content = result.get('content', [])
        if not content:
            return {'results': [], 'text': ''}
        
        text = content[0].get('text', '')
        
        # Parse structured results from text
        # (This is a simplified parser - you can enhance it)
        results = []
        current_result = {}
        
        for line in text.split('\n'):
            line = line.strip()
            if line.startswith('## [') and ']' in line:
                if current_result:
                    results.append(current_result)
                # Extract number and title
                parts = line.split(']', 1)
                if len(parts) == 2:
                    current_result = {'title': parts[1].strip()}
            elif line.startswith('**URL:**'):
                current_result['url'] = line.replace('**URL:**', '').strip()
            elif line.startswith('**Full Content:**'):
                current_result['has_full_content'] = True
            elif line.startswith('**Snippet:**'):
                current_result['snippet'] = line.replace('**Snippet:**', '').strip()
        
        if current_result:
            results.append(current_result)
        
        return {
            'results': results,
            'text': text,
            'count': len(results)
        }
    
    def stop(self):
        """Stop the MCP server"""
        if self.process:
            self.process.terminate()
            self.process.wait(timeout=5)
            self._log("info", "🛑 MCP server stopped")
    
    def __enter__(self):
        """Context manager entry"""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop()


# Example usage for integration with SearXNG
def integrate_with_searxng_cnc():
    """
    Example of how to integrate with CNSBridge
    
    Add this to your cnc.py:
    
    from searxng_mcp_client import SearXNGMCPClient
    
    # In CNSBridge.__init__
    self.mcp_client = SearXNGMCPClient(
        server_path="searxng-mcp-server",
        logger=self.get_logger()
    )
    self.mcp_client.start()
    
    # Replace web_search method
    def web_search(self, query, num_results=10):
        if self.mcp_client:
            result = self.mcp_client.search_web(
                query, 
                num_results, 
                fetch_content=True
            )
            if result:
                return result['results']
        return None
    
    # In shutdown method
    self.mcp_client.stop()
    """
    pass


if __name__ == "__main__":
    # Test the client
    import sys
    
    print("Testing SearXNG MCP Client")
    print("=" * 60)
    
    with SearXNGMCPClient() as client:
        # Test search
        print("\nTest 1: Search with full content")
        result = client.search_web("OpenAI GPT-4", num_results=3)
        if result:
            print(f"Found {result['count']} results")
            print(result['text'][:500] + "...")
        
        # Test snippet search
        print("\nTest 2: Quick snippet search")
        result = client.search_snippets_only("latest AI news", num_results=5)
        if result:
            print(f"Found {result['count']} results")
        
        # Test URL fetch
        print("\nTest 3: Fetch specific URL")
        content = client.fetch_url("https://en.wikipedia.org/wiki/Artificial_intelligence")
        if content:
            print(f"Fetched {len(content)} characters")
            print(content[:300] + "...")
    
    print("\n" + "=" * 60)
    print("Tests completed")
