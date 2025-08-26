import React, { useState, useRef } from 'react';

export interface FileLoaderProps<T> {
  name: string;
  onDataLoaded: (data: T) => Promise<string[]>;
  className?: string;
}

export const FileLoader = <T,>({
  name,
  onDataLoaded,
  className = ''
}: FileLoaderProps<T>) => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string>('');
  const [warning, setWarning] = useState<string>('');
  const [downloadLink, setDownloadLink] = useState<{href: string; filename: string} | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);

  const handleLatestClick = async () => {
    setIsLoading(true);
    setError('');
    setWarning('');
    
    try {
      const res = await fetch(`/tuning/${name}/latest.json`);

      if (res.ok) {
        const filename = res.headers.get('X-Filename');
        
        if (filename) {
          setDownloadLink({
            href: `/tuning/${name}/${filename}`,
            filename: `${name}-${filename}`
          });
        }

        const json = await res.json();
        const warnings = await onDataLoaded(json);
        
        if (warnings.length > 0) {
          setWarning(warnings.join('\n'));
        }
      } else {
        throw new Error("No data files found");
      }
    } catch (err: unknown) {
      console.error(err);
      if (typeof err === 'string') {
        setError(err);
      } else if (err instanceof Error) {
        if (import.meta.env.DEV) {
          setError(`${err}\n${err.stack}`);
        } else {
          setError(err.message);
        }
      }
    } finally {
      setIsLoading(false);
    }
  };

  const handleFileChange = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    setError('');
    setWarning('');
    
    const reader = new FileReader();
    
    reader.onload = async () => {
      if (typeof reader.result === 'string') {
        try {
          const warnings = await onDataLoaded(JSON.parse(reader.result.trim()));
          if (warnings.length > 0) {
            setWarning(warnings.join('\n'));
          }
        } catch (err: unknown) {
          console.error(err);
          if (typeof err === 'string') {
            setError(err);
          } else if (err instanceof Error) {
            if (import.meta.env.DEV) {
              setError(`${err}\n${err.stack}`);
            } else {
              setError(err.message);
            }
          }
        }
      }
    };
    
    reader.readAsText(file);
  };

  const handleBrowseClick = () => {
    fileInputRef.current?.click();
  };

  return (
    <div className={`space-y-4 ${className}`}>
      <div className="flex gap-4">
        <button
          onClick={handleLatestClick}
          disabled={isLoading}
          className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:bg-gray-400"
        >
          {isLoading ? 'Loading...' : 'Load Latest'}
        </button>
        
        <button
          onClick={handleBrowseClick}
          className="px-4 py-2 bg-green-500 text-white rounded hover:bg-green-600"
        >
          Browse Files
        </button>
        
        <input
          ref={fileInputRef}
          type="file"
          accept=".json"
          onChange={handleFileChange}
          className="hidden"
        />
      </div>

      {downloadLink && (
        <div className="mt-4">
          <a
            href={downloadLink.href}
            download={downloadLink.filename}
            className="inline-block px-4 py-2 bg-gray-500 text-white rounded hover:bg-gray-600"
          >
            Download
          </a>
        </div>
      )}

      {error && (
        <div className="p-3 bg-red-100 border border-red-400 text-red-700 rounded">
          <pre className="whitespace-pre-wrap">{error}</pre>
        </div>
      )}

      {warning && (
        <div className="p-3 bg-yellow-100 border border-yellow-400 text-yellow-700 rounded">
          <pre className="whitespace-pre-wrap">{warning}</pre>
        </div>
      )}
    </div>
  );
};

export default FileLoader; 