#!/usr/bin/env node

/**
 * SearXNG MCP Server
 * 
 * Custom MCP server for SearXNG web search with full content fetching
 * Integrates with SearXNG and provides rich context extraction
 */

const { Server } = require('@modelcontextprotocol/sdk/server/index.js');
const { StdioServerTransport } = require('@modelcontextprotocol/sdk/server/stdio.js');
const { 
  CallToolRequestSchema, 
  ListToolsRequestSchema 
} = require('@modelcontextprotocol/sdk/types.js');

const axios = require('axios');
const cheerio = require('cheerio');
const TurndownService = require('turndown');

// Configuration
const SEARXNG_URL = process.env.SEARXNG_URL || 'http://127.0.0.1:8080';
const MAX_CONTENT_LENGTH = parseInt(process.env.MAX_CONTENT_LENGTH) || 3000;
const REQUEST_TIMEOUT = parseInt(process.env.REQUEST_TIMEOUT) || 10000;
const USER_AGENT = 'AGi/1.0 (AGi Robot Assistant)';

// Initialize turndown for HTML to Markdown conversion
const turndownService = new TurndownService({
  headingStyle: 'atx',
  codeBlockStyle: 'fenced'
});

// Create MCP server
const server = new Server(
  {
    name: 'searxng-searxng',
    version: '1.0.0',
  },
  {
    capabilities: {
      tools: {},
    },
  }
);

/**
 * Extract clean text content from HTML
 */
async function extractContent(url) {
  try {
    const response = await axios.get(url, {
      timeout: REQUEST_TIMEOUT,
      headers: {
        'User-Agent': USER_AGENT,
        'Accept': 'text/html,application/xhtml+xml',
        'Accept-Language': 'en-US,en;q=0.9',
      },
      maxRedirects: 5,
    });

    const $ = cheerio.load(response.data);
    
    // Remove unwanted elements
    $('script, style, nav, header, footer, aside, .ad, .advertisement, #comments').remove();
    
    // Try to find main content
    let mainContent = '';
    
    // Strategy 1: Look for article/main tags
    const article = $('article, main, [role="main"]').first();
    if (article.length) {
      mainContent = article.html();
    } else {
      // Strategy 2: Find largest content block
      let maxLength = 0;
      $('div, section').each((i, elem) => {
        const text = $(elem).text().trim();
        if (text.length > maxLength) {
          maxLength = text.length;
          mainContent = $(elem).html();
        }
      });
    }
    
    // Convert to markdown for better structure
    const markdown = turndownService.turndown(mainContent || $.html());
    
    // Clean up the text
    const cleaned = markdown
      .split('\n')
      .map(line => line.trim())
      .filter(line => line.length > 0)
      .join('\n')
      .substring(0, MAX_CONTENT_LENGTH);
    
    return {
      success: true,
      content: cleaned,
      length: cleaned.length,
    };
    
  } catch (error) {
    return {
      success: false,
      error: error.message,
      content: '',
    };
  }
}

/**
 * Search SearXNG
 */
async function searchSearXNG(query, numResults = 10) {
  try {
    const response = await axios.get(`${SEARXNG_URL}/search`, {
      params: {
        q: query,
        format: 'json',
        categories: 'general',
      },
      timeout: REQUEST_TIMEOUT,
    });
    
    const results = response.data.results.slice(0, numResults);
    return { success: true, results };
    
  } catch (error) {
    return { 
      success: false, 
      error: error.message,
      results: [] 
    };
  }
}

/**
 * Enhanced search with content extraction
 */
async function enhancedSearch(query, numResults = 10, fetchContent = true) {
  // First, get search results
  const searchResponse = await searchSearXNG(query, numResults);
  
  if (!searchResponse.success) {
    return {
      error: `Search failed: ${searchResponse.error}`,
      results: []
    };
  }
  
  const results = searchResponse.results;
  
  // If content fetching disabled, return snippets only
  if (!fetchContent) {
    return {
      query,
      count: results.length,
      results: results.map((r, i) => ({
        number: i + 1,
        title: r.title || 'No title',
        url: r.url || '',
        snippet: r.content || '',
        engine: r.engine || 'unknown',
      }))
    };
  }
  
  // Fetch full content for each result
  const enrichedResults = await Promise.all(
    results.map(async (r, i) => {
      const contentResult = await extractContent(r.url);
      
      return {
        number: i + 1,
        title: r.title || 'No title',
        url: r.url || '',
        snippet: r.content || '',
        engine: r.engine || 'unknown',
        full_content: contentResult.success ? contentResult.content : '',
        content_length: contentResult.success ? contentResult.length : 0,
        fetch_error: contentResult.success ? null : contentResult.error,
      };
    })
  );
  
  return {
    query,
    count: enrichedResults.length,
    searxng_url: SEARXNG_URL,
    results: enrichedResults
  };
}

/**
 * Fetch content from specific URL
 */
async function fetchURL(url) {
  const result = await extractContent(url);
  
  if (result.success) {
    return {
      url,
      content: result.content,
      length: result.length,
    };
  } else {
    return {
      url,
      error: result.error,
      content: '',
    };
  }
}

// Register tools
server.setRequestHandler(ListToolsRequestSchema, async () => {
  return {
    tools: [
      {
        name: 'search_web',
        description: 'Search the web using SearXNG and optionally fetch full page content. Returns search results with titles, URLs, snippets, and full content if requested.',
        inputSchema: {
          type: 'object',
          properties: {
            query: {
              type: 'string',
              description: 'The search query',
            },
            num_results: {
              type: 'number',
              description: 'Number of results to return (default: 10, max: 20)',
              default: 10,
            },
            fetch_content: {
              type: 'boolean',
              description: 'Whether to fetch full page content (default: true)',
              default: true,
            },
          },
          required: ['query'],
        },
      },
      {
        name: 'fetch_url',
        description: 'Fetch and extract clean content from a specific URL. Removes ads, navigation, and other clutter.',
        inputSchema: {
          type: 'object',
          properties: {
            url: {
              type: 'string',
              description: 'The URL to fetch content from',
            },
          },
          required: ['url'],
        },
      },
      {
        name: 'search_snippets_only',
        description: 'Fast search that returns only snippets without fetching full content. Use when you need quick overview.',
        inputSchema: {
          type: 'object',
          properties: {
            query: {
              type: 'string',
              description: 'The search query',
            },
            num_results: {
              type: 'number',
              description: 'Number of results to return',
              default: 10,
            },
          },
          required: ['query'],
        },
      },
    ],
  };
});

// Handle tool calls
server.setRequestHandler(CallToolRequestSchema, async (request) => {
  const { name, arguments: args } = request.params;
  
  try {
    switch (name) {
      case 'search_web': {
        const query = args.query;
        const numResults = Math.min(args.num_results || 10, 20);
        const fetchContent = args.fetch_content !== false;
        
        const result = await enhancedSearch(query, numResults, fetchContent);
        
        // Format results for LLM
        let formattedText = `# Search Results for: "${query}"\n\n`;
        formattedText += `Found ${result.count} results\n\n`;
        
        result.results.forEach((r) => {
          formattedText += `## [${r.number}] ${r.title}\n`;
          formattedText += `**URL:** ${r.url}\n`;
          formattedText += `**Source:** ${r.engine}\n\n`;
          
          if (r.full_content) {
            formattedText += `**Full Content:**\n${r.full_content}\n\n`;
          } else if (r.snippet) {
            formattedText += `**Snippet:** ${r.snippet}\n\n`;
          }
          
          if (r.fetch_error) {
            formattedText += `*(Could not fetch full content: ${r.fetch_error})*\n\n`;
          }
          
          formattedText += '---\n\n';
        });
        
        return {
          content: [
            {
              type: 'text',
              text: formattedText,
            },
          ],
        };
      }
      
      case 'fetch_url': {
        const url = args.url;
        const result = await fetchURL(url);
        
        let text = `# Content from: ${url}\n\n`;
        
        if (result.content) {
          text += result.content;
        } else {
          text += `Error fetching content: ${result.error}`;
        }
        
        return {
          content: [
            {
              type: 'text',
              text,
            },
          ],
        };
      }
      
      case 'search_snippets_only': {
        const query = args.query;
        const numResults = Math.min(args.num_results || 10, 20);
        
        const result = await enhancedSearch(query, numResults, false);
        
        let formattedText = `# Quick Search: "${query}"\n\n`;
        
        result.results.forEach((r) => {
          formattedText += `[${r.number}] **${r.title}**\n`;
          formattedText += `${r.url}\n`;
          formattedText += `${r.snippet}\n\n`;
        });
        
        return {
          content: [
            {
              type: 'text',
              text: formattedText,
            },
          ],
        };
      }
      
      default:
        throw new Error(`Unknown tool: ${name}`);
    }
  } catch (error) {
    return {
      content: [
        {
          type: 'text',
          text: `Error executing ${name}: ${error.message}`,
        },
      ],
      isError: true,
    };
  }
});

// Start server
async function main() {
  const transport = new StdioServerTransport();
  await server.connect(transport);
  
  console.error('SearXNG MCP Server started');
  console.error(`SearXNG URL: ${SEARXNG_URL}`);
  console.error(`Max content length: ${MAX_CONTENT_LENGTH}`);
}

main().catch((error) => {
  console.error('Server error:', error);
  process.exit(1);
});
