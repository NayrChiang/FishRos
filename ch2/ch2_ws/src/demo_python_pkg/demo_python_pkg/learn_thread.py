import threading
import requests

class Download:
    def download(self, url, callback_word_count):
        print(f"Thread: {threading.get_ident()} Start downloading: {url}")
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback_word_count(url, response.text)

    def start_download(self, url, callback_word_count):
        self.download(url, callback_word_count)
        thread = threading.Thread(target=self.download, args=(url, callback_word_count))
        thread.start()


def word_count(url, result):
    print(f"{url}: {len(result)}->{result[:5]}")

def main():
    download = Download()
    download.start_download("http://127.0.0.1:8000/novel1.txt", word_count)
    download.start_download("http://127.0.0.1:8000/novel2.txt", word_count)
    download.start_download("http://127.0.0.1:8000/novel3.txt", word_count)