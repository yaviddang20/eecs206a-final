const fs = require("fs");
const path = require("path");
const { Readable } = require("stream");

async function main() {
  const { loadPly, serializeSpz } = await import("spz-js");

  const inputPly = process.argv[2];
  const outputSpz = process.argv[3] ||
    inputPly.replace(/\.ply$/i, ".spz");

  if (!inputPly) {
    console.error("Usage: node convert.js input.ply output.spz");
    process.exit(1);
  }

  const nodeStream = fs.createReadStream(inputPly);
  const webStream = Readable.toWeb(nodeStream);

  const gs = await loadPly(webStream);
  const spzData = await serializeSpz(gs);

  fs.writeFileSync(outputSpz, Buffer.from(spzData));
  console.log(`Converted ${inputPly} → ${outputSpz}`);
}

main().catch(err => {
  console.error(err);
  process.exit(1);
});
