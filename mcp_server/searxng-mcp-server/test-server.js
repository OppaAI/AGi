#!/usr/bin/env node

/**
 * Test script for SearXNG MCP Server
 * Tests all available tools
 */

const { spawn } = require('child_process');
const readline = require('readline');

// Test queries
const tests = [
  {
    name: 'Search with full content',
    request: {
      jsonrpc: '2.0',
      id: 1,
      method: 'tools/call',
      params: {
        name: 'search_web',
        arguments: {
          query: 'OpenAI GPT-4',
          num_results: 3,
          fetch_content: true
        }
      }
    }
  },
  {
    name: 'Quick search (snippets only)',
    request: {
      jsonrpc: '2.0',
      id: 2,
      method: 'tools/call',
      params: {
        name: 'search_snippets_only',
        arguments: {
          query: 'latest AI news',
          num_results: 5
        }
      }
    }
  },
  {
    name: 'Fetch specific URL',
    request: {
      jsonrpc: '2.0',
      id: 3,
      method: 'tools/call',
      params: {
        name: 'fetch_url',
        arguments: {
          url: 'https://en.wikipedia.org/wiki/Artificial_intelligence'
        }
      }
    }
  }
];

async function runTest(test) {
  return new Promise((resolve, reject) => {
    console.log(`\n${'='.repeat(60)}`);
    console.log(`Testing: ${test.name}`);
    console.log(`${'='.repeat(60)}\n`);
    
    const server = spawn('node', ['searxng-mcp-server.js'], {
      stdio: ['pipe', 'pipe', 'pipe'],
      env: {
        ...process.env,
        SEARXNG_URL: process.env.SEARXNG_URL || 'http://127.0.0.1:8080'
      }
    });
    
    let output = '';
    
    server.stdout.on('data', (data) => {
      output += data.toString();
      
      // Check if we have a complete JSON response
      try {
        const lines = output.split('\n').filter(l => l.trim());
        for (const line of lines) {
          const response = JSON.parse(line);
          if (response.id === test.request.id) {
            console.log('Response received:');
            console.log(JSON.stringify(response, null, 2));
            server.kill();
            resolve(response);
            return;
          }
        }
      } catch (e) {
        // Not yet a complete response
      }
    });
    
    server.stderr.on('data', (data) => {
      console.error('Server log:', data.toString());
    });
    
    server.on('error', (error) => {
      reject(error);
    });
    
    // Wait for server to start, then send request
    setTimeout(() => {
      console.log('Sending request:', JSON.stringify(test.request, null, 2));
      server.stdin.write(JSON.stringify(test.request) + '\n');
    }, 1000);
    
    // Timeout after 30 seconds
    setTimeout(() => {
      server.kill();
      reject(new Error('Test timeout'));
    }, 30000);
  });
}

async function main() {
  console.log('MCP SearXNG Server Test Suite');
  console.log(`SearXNG URL: ${process.env.SEARXNG_URL || 'http://127.0.0.1:8080'}`);
  
  let passed = 0;
  let failed = 0;
  
  for (const test of tests) {
    try {
      await runTest(test);
      passed++;
      console.log(`✅ ${test.name} - PASSED`);
    } catch (error) {
      failed++;
      console.error(`❌ ${test.name} - FAILED:`, error.message);
    }
  }
  
  console.log(`\n${'='.repeat(60)}`);
  console.log('Test Results:');
  console.log(`✅ Passed: ${passed}`);
  console.log(`❌ Failed: ${failed}`);
  console.log(`${'='.repeat(60)}\n`);
  
  process.exit(failed > 0 ? 1 : 0);
}

main();
