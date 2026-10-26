"""
Cloudflare R2 (S3-compatible object storage) for the rendered MP4s: free up to 10 GB, and no charge for downloads.

The bucket stays private.  The site hands out short-lived signed links, so a video is only reachable by someone who already got past
the login.  boto3 is imported only when R2 is configured.
"""
from __future__ import annotations

from .config import Settings


class R2:
    def __init__(self, settings: Settings, client=None) -> None:
        self.settings = settings
        self.enabled = settings.r2_enabled
        self._client = client                                   # tests inject a fake

    @property
    def client(self):
        if self._client is None:
            import boto3
            from botocore.config import Config
            self._client = boto3.client(
                "s3", endpoint_url=self.settings.r2_endpoint, region_name="auto",
                aws_access_key_id=self.settings.r2_access_key_id, aws_secret_access_key=self.settings.r2_secret_access_key,
                config=Config(signature_version="s3v4", retries={"max_attempts": 3}))
        return self._client

    def upload_file(self, path, key: str, content_type: str = "video/mp4") -> None:
        self.client.upload_file(str(path), self.settings.r2_bucket, key, ExtraArgs={"ContentType": content_type})

    def presigned_url(self, key: str, seconds: int = 3600) -> str:
        return self.client.generate_presigned_url(
            "get_object", Params={"Bucket": self.settings.r2_bucket, "Key": key}, ExpiresIn=seconds)

    def delete(self, key: str) -> None:
        self.client.delete_object(Bucket=self.settings.r2_bucket, Key=key)
