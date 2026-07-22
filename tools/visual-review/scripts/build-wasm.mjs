import { mkdirSync } from 'node:fs'
import { dirname, resolve } from 'node:path'
import { fileURLToPath } from 'node:url'
import { spawnSync } from 'node:child_process'

const here = dirname(fileURLToPath(import.meta.url))
const project = resolve(here, '..')
const repository = resolve(project, '../..')
const output = resolve(project, 'src/generated')
const wasm = resolve(
  repository,
  'target/wasm32-unknown-unknown/release/schemweave_review_wasm.wasm',
)

function run(command, args) {
  const result = spawnSync(command, args, {
    cwd: repository,
    stdio: 'inherit',
  })
  if (result.status !== 0) process.exit(result.status ?? 1)
}

mkdirSync(output, { recursive: true })
run('cargo', [
  'build',
  '--locked',
  '--release',
  '--package',
  'schemweave-review-wasm',
  '--target',
  'wasm32-unknown-unknown',
])
run('wasm-bindgen', [
  wasm,
  '--target',
  'web',
  '--out-dir',
  output,
  '--out-name',
  'schemweave_review_wasm',
  '--typescript',
])
