import { createReadStream, existsSync } from 'node:fs'
import { resolve } from 'node:path'
import { defineConfig, type Connect, type Plugin } from 'vite'

const filenames = new Set(['corpus.json', 'elk.json'])

function reviewData(dataDirectory: string): Plugin {
  const middleware: Connect.NextHandleFunction = (request, response, next) => {
    const filename = request.url?.match(/^\/review-data\/(corpus|elk)\.json$/)?.[0]
      ?.split('/')
      .at(-1)
    if (!filename || !filenames.has(filename)) {
      next()
      return
    }
    const path = resolve(dataDirectory, filename)
    if (!existsSync(path)) {
      response.statusCode = 404
      response.end(`Missing ${path}`)
      return
    }
    response.setHeader('Content-Type', 'application/json')
    createReadStream(path).pipe(response)
  }

  return {
    name: 'schemweave-review-data',
    configureServer(server) {
      server.middlewares.use(middleware)
    },
    configurePreviewServer(server) {
      server.middlewares.use(middleware)
    },
  }
}

export default defineConfig(() => {
  const dataDirectory =
    process.env.SCHEMWEAVE_REVIEW_DATA_DIR ?? resolve(import.meta.dirname, '.review-data')
  return {
    plugins: [reviewData(dataDirectory)],
    worker: { format: 'es' as const },
  }
})
